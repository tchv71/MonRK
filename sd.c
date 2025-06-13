/*
It is an open source software to implement SD routines to
small embedded systems. This is a free software and is opened for education,
research and commercial developments under license policy of following trems.

(C) 2013 vinxru (aleksey.f.morozov@gmail.com)

It is a free software and there is NO WARRANTY.
No restriction on use. You can use, modify and redistribute it for
personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
Redistributions of source code must retain the above copyright notice.

Version 0.99 5-05-2013
*/

#include "common.h"
// #include <delay.h>
#include "sd.h"
#include "fs.h"
#include <hardware/spi.h>
#include <hardware/gpio.h>
#include "spi_sd.h"

BYTE sd_sdhc; /* Используется SDHC карта */

/**************************************************************************
 *  Протокол SPI для Pico                                               *
 *  Может отличаться для разных МК.                                        *
 **************************************************************************/

/* Куда подключена линия CS карты */
#define SPI_FILL_CHAR 0xFF

static void SD_CS_ENABLE()
{
  gpio_put(SPI_CSn, 0);
  uint8_t fill = SPI_FILL_CHAR;
  spi_write_blocking(_SPI, &fill, 1);
}

static void SD_CS_DISABLE()
{
  gpio_put(SPI_CSn, 1);
  uint8_t fill = SPI_FILL_CHAR;
  spi_write_blocking(_SPI, &fill, 1);
}
/* Совместимость с разными версиями CodeVisionAVR */
#ifndef SPI2X
#define SPI2X 0
#endif

#if 0
#define SPI_INIT \
  {              \
    SPCR = 0x52; \
    SPSR = 0x00; \
  }
#define SPI_HIGHSPEED                    \
  {                                      \
    SPCR = 0x50; /*SPSR |= (1<<SPI2X);*/ \
    delay_ms(1);                         \
  }
#endif

static void spi_transmit(BYTE data)
{
  spi_write_blocking(_SPI, &data, 1);
}

static BYTE spi_receive()
{
  BYTE data;
  spi_read_blocking(_SPI, 0xff, &data, 1);
  return data;
}

/**************************************************************************
 *  Отправка команды                                                       *
 **************************************************************************/

/* Используемые команды SD карты */

#define GO_IDLE_STATE (0x40 | 0)
#define SEND_IF_COND (0x40 | 8)
#define READ_SINGLE_BLOCK (0x40 | 17)
#define WRITE_SINGLE_BLOCK (0x40 | 24)
#define SD_SEND_OP_COND (0x40 | 41)
#define APP_CMD (0x40 | 55)
#define READ_OCR (0x40 | 58)

static BYTE sd_sendCommand(BYTE cmd, DWORD arg)
{
  BYTE response, retry;

  /* Размещение этого кода тут -4 команды, хотя вроде лишине проверки */
  if (sd_sdhc == 0 && (cmd == READ_SINGLE_BLOCK || cmd == WRITE_SINGLE_BLOCK))
    arg <<= 9;

  /* Выбираем карту */
  SD_CS_ENABLE();

  /* Заголовок команды */
  spi_transmit(cmd);
  spi_transmit(((BYTE *)&arg)[3]);
  spi_transmit(((BYTE *)&arg)[2]);
  spi_transmit(((BYTE *)&arg)[1]);
  spi_transmit(((BYTE *)&arg)[0]);

  /* Пару команд требуют CRC. Остальные же команды игнорируют его, поэтому упрощаем код */
  spi_transmit(cmd == SEND_IF_COND ? 0x87 : 0x95);

  /* Ждем подтвреждение (256 тактов) */
  retry = 0;
  while ((response = spi_receive()) == 0xFF)
    if (++retry == 0)
      break;

  /* Результат команды READ_OCR обрабатываем тут, так как в конце этой функции мы снимем CS и пропускаем 1 байт */
  if (response == 0 && cmd == READ_OCR)
  {
    /* 32 бита из которых нас интересует один бит */
    sd_sdhc = spi_receive() & 0x40;
    spi_receive();
    spi_receive();
    spi_receive();
  }

  /* отпускаем CS и пауза в 1 байт*/
  SD_CS_DISABLE();
  spi_receive();

  return response;
}

/**************************************************************************
 *  Проверка готовности/наличия карты                                      *
 **************************************************************************/

BYTE sd_check()
{
  BYTE i = 0;
  do
  {
    sd_sendCommand(APP_CMD, 0);
    if (sd_sendCommand(SD_SEND_OP_COND, 0x40000000) == 0)
      return 0;
  } while (--i);
  return 1;
}

/**************************************************************************
 *  Инициализация карты (эта функция вызывается функцией sd_init)          *
 **************************************************************************/

static BYTE sd_init_int()
{
  BYTE i;

  /* Сбрасываем SDHC флаг */
  sd_sdhc = 0;

  /* Минимум 80 пустых тактов */
  for (i = 10; i; --i)
    spi_receive();

  /* CMD0 Посылаем команду сброса */
  if (sd_sendCommand(GO_IDLE_STATE, 0) != 1)
    goto abort;

  /* CMD8 Узнаем версию карты */
  i = 0;
  if (sd_sendCommand(SEND_IF_COND, 0x000001AA))
    i = 1;

  /* CMD41 Ожидание окончания инициализации */
  if (sd_check())
    goto abort;

  /* Только для второй версии карты */
  if (i)
  {
    /* CMD58 определение SDHC карты. Ответ обрабатывается в функции sd_sendCommand */
    if (sd_sendCommand(READ_OCR, 0) != 0)
      goto abort;
  }

  return 0;
abort:
  return 1;
}

/**************************************************************************
 *  Инициализация карты                                                    *
 **************************************************************************/

BYTE sd_init()
{
  BYTE tries;

  /* Освобождаем CS на всякий случай */
  SD_CS_DISABLE();

  /* Включаем _SPI */
  // SPI_INIT
  spi_set_baudrate(_SPI, 400 * 1000);
  /* Делаем несколько попыток инициализации */
  tries = 10;
  while (sd_init_int())
    if (--tries == 0)
    {
      lastError = ERR_DISK_ERR;
      return 1;
    }

  /* Включаем максимальную скорость */
  // SPI_HIGHSPEED
  spi_set_baudrate(_SPI, 10 * 1000 * 1000);

  return 0;
}

/**************************************************************************
 *  Ожидание определенного байта на шине                                   *
 **************************************************************************/

static BYTE sd_waitBus(BYTE byte)
{
  WORD retry = 0;
  do
  {
    if (spi_receive() == byte)
      return 0;
  } while (++retry);
  return 1;
}

/**************************************************************************
 *  Чтение произвольного участка сектора                                   *
 **************************************************************************/

BYTE sd_read(BYTE *buffer, DWORD sector, WORD offsetInSector, WORD length)
{
  BYTE b;
  WORD i;

  /* Посылаем команду */
  if (sd_sendCommand(READ_SINGLE_BLOCK, sector))
    goto abort;

  /* Сразу же возращаем CS, что бы принять ответ команды */
  SD_CS_ENABLE();

  /* Ждем стартовый байт */
  if (sd_waitBus(0xFE))
    goto abort;

  /* Принимаем 512 байт */
  for (i = 512; i; --i)
  {
    b = spi_receive();
    if (offsetInSector)
    {
      offsetInSector--;
      continue;
    }
    if (length == 0)
      continue;
    length--;
    *buffer++ = b;
  }

  /* CRC игнорируем */
  spi_receive();
  spi_receive();

  /* отпускаем CS и пауза в 1 байт*/
  SD_CS_DISABLE();
  spi_receive();

  /* Ок */
  return 0;

  /* Ошибка и отпускаем CS.*/
abort:
  SD_CS_DISABLE();
  lastError = ERR_DISK_ERR;
  return 1;
}

/**************************************************************************
 *  Запись сектора (512 байт)                                              *
 **************************************************************************/

BYTE sd_write512(BYTE *buffer, DWORD sector)
{
  WORD n;

  /* Посылаем команду */
  if (sd_sendCommand(WRITE_SINGLE_BLOCK, sector))
    goto abort;

  /* Сразу же возращаем CS, что бы отправить блок данных */
  SD_CS_ENABLE();

  /* Посылаем стартовый байт */
  spi_transmit(0xFE);

  /* Данные */
  for (n = 512; n; --n)
    spi_transmit(*buffer++);

  /* CRC игнорируется */
  spi_transmit(0xFF);
  spi_transmit(0xFF);

  /* Ответ МК */
  if ((spi_receive() & 0x1F) != 0x05)
    goto abort;

  /* Ждем окончания записи, т.е. пока не освободится шина */
  if (sd_waitBus(0xFF))
    goto abort;

  /* отпускаем CS и пауза в 1 байт*/
  SD_CS_DISABLE();
  spi_receive();

  /* Ок */
  return 0;

  /* Ошибка.*/
abort:
  SD_CS_DISABLE();
  lastError = ERR_DISK_ERR;
  return 1;
}

/**
 * @brief  Initializes the SD/SD communication.
 * @param  None
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_Init(void)
{
  return sd_init();
}

/**
 * @brief  Detect if SD card is correctly plugged in the memory slot.
 * @param  None
 * @retval Return if SD is detected or not
 */
uint8_t SD_Detect(void)
{
  uint8_t status = SD_PRESENT;

  /*!< Check GPIO to detect SD */
  //  if (GPIO_ReadInputData(SD_DETECT_GPIO_PORT) & SD_DETECT_PIN)
  //  {
  //    status = SD_NOT_PRESENT;
  //  }
  return status;
}

/**
 * @brief  Returns information about specific card.
 * @param  cardinfo: pointer to a SD_CardInfo structure that contains all SD
 *         card information.
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
  SD_Error status = SD_RESPONSE_FAILURE;

  status = SD_GetCSDRegister(&(cardinfo->SD_csd));
  status = SD_GetCIDRegister(&(cardinfo->SD_cid));
  cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1);
  cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
  cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
  // cardinfo->CardCapacity *= cardinfo->CardBlockSize;

  /*!< Returns the reponse */
  return status;
}

/**
 * @brief  Reads a block of data from the SD.
 * @param  pBuffer: pointer to the buffer that receives the data read from the
 *                  SD.
 * @param  ReadAddr: SD's internal address to read from.
 * @param  BlockSize: the SD card Data block size.
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_ReadBlock(uint8_t *pBuffer,  uint32_t ReadAddr, uint32_t ofs, uint16_t BlockSize)
{
  return sd_read(pBuffer, ReadAddr, ofs, BlockSize);
#if 0
  uint32_t i = 0;
  SD_Error rvalue = SD_RESPONSE_FAILURE;

  /*!< SD chip select low */
  SD_CS_ENABLE();

  /*!< Send CMD17 (SD_CMD_READ_SINGLE_BLOCK) to read one block */
  SD_SendCmd(SD_CMD_READ_SINGLE_BLOCK, ReadAddr, 0xFF);

  /*!< Check if the SD acknowledged the read block command: R1 response (0x00: no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
  {
    /*!< Now look for the data token to signify the start of the data */
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
    {
      /*!< Read the SD block data : read NumByteToRead data */
      for (i = 0; i < BlockSize; i++)
      {
        /*!< Save the received data */
        *pBuffer = SD_ReadByte();

        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
      }
      /*!< Get CRC bytes (not really needed by us, but required by SD) */
      SD_ReadByte();
      SD_ReadByte();
      /*!< Set response value to success */
      rvalue = SD_RESPONSE_NO_ERROR;
    }
  }
  /*!< SD chip select high */
  SD_CS_DISABLE();

  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);

  /*!< Returns the reponse */
  return rvalue;
#endif
}

/**
 * @brief  Reads multiple block of data from the SD.
 * @param  pBuffer: pointer to the buffer that receives the data read from the
 *                  SD.
 * @param  ReadSector: SD's sector to read from.
 * @param  BlockSize: the SD card Data block size.
 * @param  NumberOfBlocks: number of blocks to be read.
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_ReadMultiBlocks(uint8_t *pBuffer, uint32_t ReadSector, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  SD_Error rvalue = SD_RESPONSE_FAILURE;

  /*!< SD chip select low */
  // SD_CS_LOW();
  /*!< Data transfer */
  while (NumberOfBlocks--)
  {
    rvalue = sd_read(pBuffer, ReadSector, 0, BlockSize);
    if (rvalue)
      return rvalue;
    pBuffer += BlockSize;
    ReadSector += BlockSize;
  }
  return rvalue;
}

/**
 * @brief  Writes many blocks on the SD
 * @param  pBuffer: pointer to the buffer containing the data to be written on
 *                  the SD.
 * @param  WriteSector: address to write on (512 byte sector number).
 * @param  BlockSize: the SD card Data block size.
 * @param  NumberOfBlocks: number of blocks to be written.
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_WriteMultiBlocks(uint8_t *pBuffer, uint32_t WriteSector, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  SD_Error rvalue = SD_RESPONSE_FAILURE;

  /*!< SD chip select low */
  SD_CS_ENABLE();
  /*!< Data transfer */
  while (NumberOfBlocks--)
  {
    rvalue = sd_write512(pBuffer, WriteSector);
    if (rvalue)
      return rvalue;
    pBuffer += BlockSize;
    ++WriteSector;
  }
  return rvalue;
}

/**
 * @brief  Read the CSD card register.
 *         Reading the contents of the CSD register in SPI mode is a simple
 *         read-block transaction.
 * @param  SD_csd: pointer on an SCD register structure
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_GetCSDRegister(SD_CSD *SD_csd)
{

  SD_Error rvalue = SD_RESPONSE_FAILURE;
  uint8_t CSD_Tab[16];
#if 0
  uint32_t i = 0;
  /*!< SD chip select low */
  SD_CS_LOW();
  /*!< Send CMD9 (CSD register) or CMD10(CSD register) */
  SD_SendCmd(SD_CMD_SEND_CSD | 0x40, 0, 0xFF);
  /*!< Wait for response in the R1 format (0x00 is no errors) */
  if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
  {
    if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
    {
      for (i = 0; i < 16; i++)
      {
        /*!< Store CSD register value on CSD_Tab */
        CSD_Tab[i] = SD_ReadByte();
      }
    }
    /*!< Get CRC bytes (not really needed by us, but required by SD) */
    SD_WriteByte(SD_DUMMY_BYTE);
    SD_WriteByte(SD_DUMMY_BYTE);
    /*!< Set response value to success */
    rvalue = SD_RESPONSE_NO_ERROR;
  }
  /*!< SD chip select high */
  SD_CS_HIGH();
  /*!< Send dummy byte: 8 Clock pulses of delay */
  SD_WriteByte(SD_DUMMY_BYTE);
#else
  rvalue = sd_sendCommand(SD_CMD_SEND_CSD | 0x40, (DWORD)CSD_Tab);
#endif

  /*!< Byte 0 */
  SD_csd->CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
  if (SD_csd->CSDStruct == 0)
  {
    SD_csd->SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
    SD_csd->Reserved1 = CSD_Tab[0] & 0x03;

    /*!< Byte 1 */
    SD_csd->TAAC = CSD_Tab[1];

    /*!< Byte 2 */
    SD_csd->NSAC = CSD_Tab[2];

    /*!< Byte 3 */
    SD_csd->MaxBusClkFrec = CSD_Tab[3];

    /*!< Byte 4 */
    SD_csd->CardComdClasses = CSD_Tab[4] << 4;

    /*!< Byte 5 */
    SD_csd->CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
    SD_csd->RdBlockLen = CSD_Tab[5] & 0x0F;

    /*!< Byte 6 */
    SD_csd->PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
    SD_csd->WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
    SD_csd->RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
    SD_csd->DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
    SD_csd->Reserved2 = 0; /*!< Reserved */

    SD_csd->DeviceSize = (CSD_Tab[6] & 0x03) << 10;

    /*!< Byte 7 */
    SD_csd->DeviceSize |= (CSD_Tab[7]) << 2;

    /*!< Byte 8 */
    SD_csd->DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;

    SD_csd->MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
    SD_csd->MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);

    /*!< Byte 9 */
    SD_csd->MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
    SD_csd->MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
    SD_csd->DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
    /*!< Byte 10 */
    SD_csd->DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;

    SD_csd->EraseGrSize = (CSD_Tab[10] & 0x40) >> 6;
    SD_csd->EraseGrMul = (CSD_Tab[10] & 0x3F) << 1;

    /*!< Byte 11 */
    SD_csd->EraseGrMul |= (CSD_Tab[11] & 0x80) >> 7;
    SD_csd->WrProtectGrSize = (CSD_Tab[11] & 0x7F);

    /*!< Byte 12 */
    SD_csd->WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
    SD_csd->ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
    SD_csd->WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
    SD_csd->MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;

    /*!< Byte 13 */
    SD_csd->MaxWrBlockLen |= (CSD_Tab[13] & 0xC0) >> 6;
    SD_csd->WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
    SD_csd->Reserved3 = 0;
    SD_csd->ContentProtectAppli = (CSD_Tab[13] & 0x01);

    /*!< Byte 14 */
    SD_csd->FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
    SD_csd->CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
    SD_csd->PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
    SD_csd->TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
    SD_csd->FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
    SD_csd->ECC = (CSD_Tab[14] & 0x03);

    /*!< Byte 15 */
    SD_csd->CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
    SD_csd->Reserved4 = 1;
  }
  else if (SD_csd->CSDStruct == 1)
  {
    SD_csd->SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
    SD_csd->Reserved1 = CSD_Tab[0] & 0x03;

    /*!< Byte 1 */
    SD_csd->TAAC = CSD_Tab[1];

    /*!< Byte 2 */
    SD_csd->NSAC = CSD_Tab[2];

    /*!< Byte 3 */
    SD_csd->MaxBusClkFrec = CSD_Tab[3];

    /*!< Byte 4 */
    SD_csd->CardComdClasses = CSD_Tab[4] << 4;

    /*!< Byte 5 */
    SD_csd->CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
    SD_csd->RdBlockLen = CSD_Tab[5] & 0x0F;

    /*!< Byte 6 */
    SD_csd->PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
    SD_csd->WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
    SD_csd->RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
    SD_csd->DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
    SD_csd->Reserved2 = 0; /*!< Reserved */

    SD_csd->DeviceSize = (CSD_Tab[6] & 0xF) << 24;

    /*!< Byte 7 */
    SD_csd->DeviceSize |= (CSD_Tab[7]) << 16;

    /*!< Byte 8 */
    SD_csd->DeviceSize |= (CSD_Tab[8]) << 8;

    SD_csd->DeviceSize |= (CSD_Tab[8]);

    // SD_csd->MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
    // SD_csd->MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);

    /*!< Byte 9 */
    // SD_csd->MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
    // SD_csd->MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
    SD_csd->DeviceSizeMul = 17 - 9;
    /*!< Byte 10 */
    // SD_csd->DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;

    SD_csd->EraseGrSize = (CSD_Tab[10] & 0x40) >> 6;
    SD_csd->EraseGrMul = (CSD_Tab[10] & 0x3F) << 1;

    /*!< Byte 11 */
    SD_csd->EraseGrMul |= (CSD_Tab[11] & 0x80) >> 7;
    SD_csd->WrProtectGrSize = (CSD_Tab[11] & 0x7F);

    /*!< Byte 12 */
    SD_csd->WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
    SD_csd->ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
    SD_csd->WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
    SD_csd->MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;

    /*!< Byte 13 */
    SD_csd->MaxWrBlockLen |= (CSD_Tab[13] & 0xC0) >> 6;
    SD_csd->WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
    SD_csd->Reserved3 = 0;
    SD_csd->ContentProtectAppli = (CSD_Tab[13] & 0x01);

    /*!< Byte 14 */
    SD_csd->FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
    SD_csd->CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
    SD_csd->PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
    SD_csd->TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
    SD_csd->FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
    SD_csd->ECC = (CSD_Tab[14] & 0x03);

    /*!< Byte 15 */
    SD_csd->CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
    SD_csd->Reserved4 = 1;
  }
  /*!< Return the response */
  return rvalue;
}

/**
 * @brief  Read the CID card register.
 *         Reading the contents of the CID register in SPI mode is a simple
 *         read-block transaction.
 * @param  SD_cid: pointer on an CID register structure
 * @retval The SD Response:
 *         - SD_RESPONSE_FAILURE: Sequence failed
 *         - SD_RESPONSE_NO_ERROR: Sequence succeed
 */
SD_Error SD_GetCIDRegister(SD_CID *SD_cid)
{
  SD_Error rvalue = SD_RESPONSE_FAILURE;
  uint8_t CID_Tab[16];

#if 0
    /*!< SD chip select low */
    SD_CS_LOW();

    /*!< Send CMD10 (CID register) */
    SD_SendCmd(SD_CMD_SEND_CID, 0, 0xFF);

    /*!< Wait for response in the R1 format (0x00 is no errors) */
    if (!SD_GetResponse(SD_RESPONSE_NO_ERROR))
    {
      if (!SD_GetResponse(SD_START_DATA_SINGLE_BLOCK_READ))
      {
	/*!< Store CID register value on CID_Tab */
	for (i = 0; i < 16; i++)
	{
	  CID_Tab[i] = SD_ReadByte();
	}
      }
      /*!< Get CRC bytes (not really needed by us, but required by SD) */
      SD_WriteByte(SD_DUMMY_BYTE);
      SD_WriteByte(SD_DUMMY_BYTE);
      /*!< Set response value to success */
      rvalue = SD_RESPONSE_NO_ERROR;
    }
    /*!< SD chip select high */
    SD_CS_HIGH();
    /*!< Send dummy byte: 8 Clock pulses of delay */
    SD_WriteByte(SD_DUMMY_BYTE);
#else
  rvalue = sd_sendCommand(SD_CMD_SEND_CID | 0x40, (DWORD)CID_Tab);
#endif

  /*!< Byte 0 */
  SD_cid->ManufacturerID = CID_Tab[0];

  /*!< Byte 1 */
  SD_cid->OEM_AppliID = CID_Tab[1] << 8;

  /*!< Byte 2 */
  SD_cid->OEM_AppliID |= CID_Tab[2];

  /*!< Byte 3 */
  SD_cid->ProdName1 = CID_Tab[3] << 24;

  /*!< Byte 4 */
  SD_cid->ProdName1 |= CID_Tab[4] << 16;

  /*!< Byte 5 */
  SD_cid->ProdName1 |= CID_Tab[5] << 8;

  /*!< Byte 6 */
  SD_cid->ProdName1 |= CID_Tab[6];

  /*!< Byte 7 */
  SD_cid->ProdName2 = CID_Tab[7];

  /*!< Byte 8 */
  SD_cid->ProdRev = CID_Tab[8];

  /*!< Byte 9 */
  SD_cid->ProdSN = CID_Tab[9] << 24;

  /*!< Byte 10 */
  SD_cid->ProdSN |= CID_Tab[10] << 16;

  /*!< Byte 11 */
  SD_cid->ProdSN |= CID_Tab[11] << 8;

  /*!< Byte 12 */
  SD_cid->ProdSN |= CID_Tab[12];

  /*!< Byte 13 */
  SD_cid->Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
  SD_cid->ManufactDate = (CID_Tab[13] & 0x0F) << 8;

  /*!< Byte 14 */
  SD_cid->ManufactDate |= CID_Tab[14];

  /*!< Byte 15 */
  SD_cid->CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
  SD_cid->Reserved2 = 1;

  /*!< Return the reponse */
  return rvalue;
}
