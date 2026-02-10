// SD Controller for Computer "Radio 86RK" / "Apogee BK01"
// (c) 10-05-2014 vinxru (aleksey.f.morozov@gmail.com)

// #include <stdafx.h>

#define F_CPU 16000000UL // freq 16 MHz

#include "common.h"
#include <string.h>
#include "sd.h"
#include "fs.h"
#include "proto.h"
#include "pico/time.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "gpios.h"
#include "hardware/pio.h"
#include "hardware/rtc.h"
#include "pico/multicore.h"
#include "pico/sync.h"

#define O_OPEN 0
#define O_CREATE 1
#define O_MKDIR 2
#define O_DELETE 100
#define O_SWAP 101

#define STA_START 0x40
#define STA_WAIT 0x41
#define STA_OK_DISK 0x42
#define STA_OK_CMD 0x43
#define STA_OK_READ 0x44
#define STA_OK_ENTRY 0x45
#define STA_OK_WRITE 0x46
#define STA_OK_RKS 0x47
#define ERR_DATETIME 0x50
#define STA_OK_BLOCK 0x4F

__attribute__((aligned(4))) BYTE buf[1024 + 512];
__attribute__((aligned(4))) BYTE rom[512];
#define flash

/*******************************************************************************
 * Для удобства                                                                 *
 *******************************************************************************/
#if !USE_DMA
void recvBin(BYTE *d, WORD l)
{
  for (; l; --l)
  {
    *d++ = recvByte();
  }
}
#endif

void recvString()
{
  BYTE c;
  BYTE *p = buf;
  do
  {
    c = recvByte();
    if (p != buf + FS_MAXFILE)
      *p++ = c;
    else
      lastError = ERR_RECV_STRING;
  } while (c);
}

void sendBin(BYTE *p, WORD l)
{
  for (; l; l--)
    sendByte(*p++);
}

void sendBinf(const BYTE *d, BYTE l)
{
  for (; l; --l)
    sendByte(*d++);
}

/*******************************************************************************
 * Отправка всех блоков файла                                                   *
 *******************************************************************************/

WORD readLength;

void readInt(char rks)
{
  WORD readedLength, lengthFromFile;
  BYTE tmp;
  BYTE *wptr;
  WORD SEC_LEN = 512;

  if (readLength == 1024)
    SEC_LEN = readLength;

  while (readLength)
  {
    // Расчет длины блока (выравниваем чтение на сектор)
    if (fs_tell())
      return;

    readedLength = SEC_LEN - (fs_tmp % SEC_LEN);
    if (readedLength > readLength)
      readedLength = readLength;

    // Уменьшаем счетчик
    readLength -= readedLength;

    // Читаем блок
    MTX_ENTER();
    BYTE res = fs_read0(buf, readedLength);
    MTX_EXIT();
    if (res)
      return;

    // Заголовок RKS файла
    wptr = buf;
    if (rks)
    { // Если rks=1, перед вызовом надо проверить, что бы readLength>4 и fs_file.ptr=0, иначе может быть злостный сбой
      rks = 0;

      // У апогея числа перепутаны
      tmp = buf[0], buf[0] = buf[1];
      buf[1] = tmp;
      tmp = buf[2], buf[2] = buf[3];
      buf[3] = tmp;

      // Посылаем адрес загрузки
      sendByte(STA_OK_RKS);
      sendWord(*(WORD *)buf);
#if !USE_DMA
      sendByte(STA_WAIT);
#endif
      // Корректируем указатели
      wptr += 4;
      readedLength -= 4;

      // Длина из файла
      lengthFromFile = *(WORD *)(buf + 2) - *(WORD *)(buf) + 1;

      // Корректируем длину
      if (readedLength > lengthFromFile)
      {
        readedLength = lengthFromFile;
      }
      else
      {
        lengthFromFile -= readedLength;
        if (readLength > lengthFromFile)
          readLength = lengthFromFile;
      }
    }

    // Отправляем блок
    sendByte(STA_OK_BLOCK);
    sendWord(readedLength);
#if USE_DMA
    dma_send(wptr, readedLength);
#else
    sendBin(wptr, readedLength);
    sendByte(STA_WAIT);
#endif
  }

  // Если все ОК
  if (!lastError)
    sendStart(STA_OK_READ);
}

/*******************************************************************************
 * Версия команд контроллера                                                    *
 *******************************************************************************/

void cmd_ver()
{
  sendStart(1);

  // Версия + Производитель
  {
    flash char *ver = "V1.1 (DMA SIMP)";
    sendBinf(ver, 16);
  }
}

/*******************************************************************************
 * BOOT / EXEC                                                                  *
 *******************************************************************************/

void cmd_boot_exec()
{
  // Файл по умолчанию
#if !USE_DMA
  const char *bootSdbiosRk = "boot/sdbios.rk";
#else
  const char *bootSdbiosRk = "boot/sdbiosds.rkl";
#endif
  if (buf[0] == 0)
    strcpy((char *)buf, /* (const char*) (nCS_GPIO_Port->IDR & nCS_Pin) ?  "boota/sdbios.rk" :  */ bootSdbiosRk);

  // Открываем файл
  MTX_ENTER();
  BYTE res = fs_open();
  MTX_EXIT();
  if (res)
    return;

  // Максимальный размер файла
  readLength = 0xFFFF;
  if (fs_getfilesize())
    return;
  if (readLength > fs_tmp)
    readLength = (WORD)fs_tmp;

  // Файлы RK должны быть длиной >4 байт. Мы заносим в readLength = 0 и программа
  // получает STA_OK. Но так как она ждет STA_OK_RKS, это будет ошибкой
  if (readLength < 4)
    readLength = 0;

  readInt(/*rks*/ 1);
}

void cmd_boot()
{
  sendStart(STA_WAIT);
  buf[0] = 0;
  cmd_boot_exec();
}

void cmd_exec()
{
  // Прием имени файла
  recvString();

  // if ((nCS_GPIO_Port->IDR & nCS_Pin) && stricmp((char*)buf, "BOOT/SHELL.RK") == 0)
  //   strcpy((char*)buf, "BOOTA/SHELL.RK");
  // Режим передачи и подтверждение
  sendStart(STA_WAIT);
  if (lastError)
    return; // Переполнение строки

  cmd_boot_exec();
}

/*******************************************************************************
 * Начать/продолжить поиск файлов в папке                                       *
 *******************************************************************************/

typedef struct
{
  char fname[11]; // File name
  BYTE fattrib;   // Attribute
  DWORD fsize;    // File size
  union
  {
    struct
    {
      WORD ftime; // Last modified time
      WORD fdate; // Last modified date
    };
    DWORD ftimedate;
  };
} FILINFO2;

void cmd_find()
{
  WORD n = 0;
  FILINFO2 info;

  // Принимаем путь
  recvString();

  // Принимаем макс кол-во элементов
  n = recvWord();
  // n = 1000;

  // MTX_ENTER();
  // sleep_ms(100);
  // MTX_EXIT();
  // Режим передачи и подтверждение
  sendStart(STA_WAIT);
  if (lastError)
    return;

  // Открываем папку
  if (buf[0] != ':')
  {
    MTX_ENTER();
    BYTE res = fs_opendir();
    MTX_EXIT();
    if (res)
      return;
  }
  // pio_gpio_init(FIFO_PIO, PIN_DIR);
  // pio_sm_set_pins_with_mask(FIFO_PIO, dmaReadSm, DIR_MASK, DIR_MASK);
  // pio_sm_set_pindirs_with_mask(FIFO_PIO, dmaReadSm, DIR_MASK, DIR_MASK);

  for (; n; --n)
  {
    /* Читаем очередной описатель */
    MTX_ENTER();
    BYTE res = fs_readdir();
    if (res)
    {
      MTX_EXIT();
      lastError = ERR_DISK_ERR;
      return;
    }

    /* Конец */
    if (FS_DIRENTRY[0] == 0)
    {
      MTX_EXIT();
      sendByte(STA_OK_CMD);
      return;
    }

    /* Сжимаем ответ для компьютера */
    memcpy(info.fname, FS_DIRENTRY + DIR_Name, 12);
    memcpy(&info.fsize, FS_DIRENTRY + DIR_FileSize, 4);
    memcpy(&info.ftimedate, FS_DIRENTRY + DIR_WrtTime, 4);
    MTX_EXIT();
    // memcpy(memcpy(memcpy(info.fname, FS_DIRENTRY+DIR_Name, 12, FS_DIRENTRY+DIR_FileSize, 4), FS_DIRENTRY+DIR_WrtTime, 4);

    /* Отправляем */
    sendByte(STA_OK_ENTRY);
#if !USE_DMA
    sendBin((BYTE *)&info, sizeof(info));
    sendByte(STA_WAIT);
#else
    // sendBin ((BYTE*) &info, sizeof(info));
    dma_send((BYTE *)&info, sizeof(info));
#endif
  }

  /* Ограничение по размеру */
  lastError = ERR_MAX_FILES; /*! Надо опеределать, что бы не было ложных ошибок */
}

/*******************************************************************************
 * Открыть/создать файл/папку                                                   *
 *******************************************************************************/

void cmd_open()
{
  BYTE mode;

  /* Принимаем режим */
  mode = recvByte();

  // Принимаем имя файла
  recvString();

  // Режим передачи и подтверждение
  sendStart(STA_WAIT);

  // Открываем/создаем файл/папку
  MTX_ENTER();
  if (mode == O_SWAP)
  {
    fs_swap();
  }
  else if (mode == O_DELETE)
  {
    fs_delete();
  }
  else if (mode == O_OPEN)
  {
    fs_open();
  }
  else if (mode < 3)
  {
    fs_open0(mode);
  }
  else
  {
    lastError = ERR_INVALID_COMMAND;
  }
  MTX_EXIT();
  // Ок
  if (!lastError)
    sendStart(STA_OK_CMD);
}

/*******************************************************************************
 * Переместить файл/папку                                                       *
 *******************************************************************************/

void cmd_move()
{
  recvString();
  sendStart(STA_WAIT);
  MTX_ENTER();
  fs_openany();
  MTX_EXIT();
  sendStart(STA_OK_WRITE);
  recvStart();
  recvString();
  sendStart(STA_WAIT);
  MTX_ENTER();
  if (!lastError)
    fs_move0();
  MTX_EXIT();
  if (!lastError)
    sendStart(STA_OK_CMD);
}

/*******************************************************************************
 * Установить/прочитать указатель чтения                                        *
 *******************************************************************************/

void cmd_lseek()
{
  BYTE mode;
  DWORD off = 0;

  // Принимаем режим и смещение
  mode = recvByte();
  off = recvWord();
  off |= recvWord() << 16;

  // Режим передачи и подтверждение
  sendStart(STA_WAIT);

  MTX_ENTER();
  // Размер файла
  if (mode == 100)
  {
    if (fs_getfilesize())
      goto abort;
  }

  // Размер диска
  else if (mode == 101)
  {
    if (fs_gettotal())
      goto abort;
  }

  // Свободное место на диске
  else if (mode == 102)
  {
    if (fs_getfree())
      goto abort;
  }

  else
  {
    /* Устаналиваем смещение. fs_tmp сохраняется */
    if (fs_lseek(off, mode))
      goto abort;
  }
  MTX_EXIT();

  // Передаем результат
  sendByte(STA_OK_CMD);
  // sendBin((BYTE *)&fs_tmp, 4);
  sendWord(fs_tmp & 0xFFFF);
  sendWord(fs_tmp >> 16);
  lastError = 0; // На всякий случай, результат уже передан
  return;
abort:
  lastError = ERR_DISK_ERR;
  MTX_EXIT();
}

/*******************************************************************************
 * Прочитать из файла                                                           *
 *******************************************************************************/

void cmd_read()
{
  DWORD s;

  // Длина
  readLength = recvWord();

  // Режим передачи и подтверждение
  sendStart(STA_WAIT);

  MTX_ENTER();
  // Ограничиваем длину длиной файла
  if (fs_getfilesize())
  {
    MTX_EXIT();
    return;
  }
  s = fs_tmp;
  BYTE res = fs_tell();
  MTX_EXIT();
  if (res)
    return;
  s -= fs_tmp;

  if (readLength > s)
    readLength = (WORD)s;

  // Отправляем все блоки файла
  readInt(/*rks*/ 0);
}

/*******************************************************************************
 * Записать данные в файл                                                       *
 *******************************************************************************/
void cmd_write()
{
  // Аргументы
  fs_wtotal = recvWord();

  // Ответ
  sendStart(STA_WAIT);

  // Конец файла
  if (fs_wtotal == 0)
  {
    MTX_ENTER();
    fs_write_eof();
    MTX_EXIT();
    sendStart(STA_OK_CMD);
    return;
  }

  // Запись данных
  do
  {
    MTX_ENTER();
    BYTE res = fs_write_start();
    MTX_EXIT();
    if (res)
      return;

    // Принимаем от компьютера блок данных
    sendByte(STA_OK_WRITE);
    sendWord(fs_file_wlen);
#if !USE_DMA
    recvStart();
    recvBin(fs_file_wbuf, fs_file_wlen);
#else
    dma_receive(fs_file_wbuf, fs_file_wlen);
#endif
    sendStart(STA_WAIT);
    MTX_ENTER();
    res = fs_write_end();
    MTX_EXIT();
    if (res)
      return;
  } while (fs_wtotal);

  sendStart(STA_OK_CMD);
}

typedef struct
{
  uint8_t Hours; /*!< Specifies the RTC Time Hour.
                      This parameter must be a number between Min_Data = 0 and Max_Data = 12 if the RTC_HourFormat_12 is selected
                      This parameter must be a number between Min_Data = 0 and Max_Data = 23 if the RTC_HourFormat_24 is selected */

  uint8_t Minutes; /*!< Specifies the RTC Time Minutes.
                        This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t Seconds; /*!< Specifies the RTC Time Seconds.
                        This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t TimeFormat; /*!< Specifies the RTC AM/PM Time.
                           This parameter can be a value of @ref RTC_AM_PM_Definitions */

  uint32_t SubSeconds; /*!< Specifies the RTC_SSR RTC Sub Second register content.
                            This parameter corresponds to a time unit range between [0-1] Second
                            with [1 Sec / SecondFraction +1] granularity */

  uint32_t SecondFraction; /*!< Specifies the range or granularity of Sub Second register content
                                corresponding to Synchronous prescaler factor value (PREDIV_S)
                                This parameter corresponds to a time unit range between [0-1] Second
                                with [1 Sec / SecondFraction +1] granularity.
                                This field will be used only by HAL_RTC_GetTime function */

  uint32_t DayLightSaving; /*!< This interface is deprecated. To manage Daylight
                                Saving Time, please use HAL_RTC_DST_xxx functions */

  uint32_t StoreOperation; /*!< This interface is deprecated. To manage Daylight
                                Saving Time, please use HAL_RTC_DST_xxx functions */
} RTC_TimeTypeDef;

// extern RTC_HandleTypeDef hrtc;

static const uint8_t list_mth[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

uint8_t Calendar_GetDayWeek(RTC_DateTypeDef thisDate)
{
  uint8_t ret = 0;
  if (thisDate.Month < 3)
  {
    thisDate.Year -= 1;
  }
  ret = (uint8_t)((thisDate.Year + (thisDate.Year / 4) - (thisDate.Year / 100) + (thisDate.Year / 400) + list_mth[thisDate.Month - 1] + thisDate.Date) % 7);
  return (ret);
}

void toDatetime(const RTC_DateTypeDef *rt, datetime_t *t)
{
  if (!t || !rt)
    return;
  t->day = rt->Date;
  t->month = rt->Month;
  t->dotw = rt->WeekDay;
  t->year = rt->Year + 20000;
}

void toDatetimeT(const RTC_TimeTypeDef *rt, datetime_t *t)
{
  if (!t || !rt)
    return;
  t->sec = rt->Seconds;
  t->min = rt->Minutes;
  t->hour = rt->Hours;
}

void toRTC_Date(const datetime_t *t, RTC_DateTypeDef *rt)
{
  if (!t || !rt)
    return;
  rt->Date = t->day;
  rt->Month = t->month;
  rt->WeekDay = t->dotw;
  rt->Year = t->year - 2000;
}

void toRTC_Time(const datetime_t *t, RTC_TimeTypeDef *rt)
{
  if (!t || !rt)
    return;
  rt->Hours = t->hour;
  rt->Minutes = t->min;
  rt->Seconds = t->sec;
  rt->SecondFraction = 255;
  rt->SubSeconds = 0;
}
/*******************************************************************************
 * Получить дату                                                                *
 *******************************************************************************/
void cmd_get_date()
{
  sendStart(STA_WAIT);
  RTC_DateTypeDef sDate = {0};
  datetime_t t;
  if (!rtc_get_datetime(&t))
  {
    lastError = ERR_DATETIME;
    return;
  }
  toRTC_Date(&t, &sDate);
  sendBin((BYTE *)&sDate, sizeof(sDate));
  sendByte(STA_OK_CMD);
  lastError = 0; // STA_OK_CMD;
}

/*******************************************************************************
 * Установить дату                                                              *
 *******************************************************************************/
void cmd_set_date()
{
  RTC_DateTypeDef sDate = {0};
  // recvStart();
  sDate.WeekDay = recvByte();
  sDate.Month = recvByte();
  sDate.Date = recvByte();
  sDate.Year = recvByte();
  sDate.WeekDay = Calendar_GetDayWeek(sDate);

  // Режим передачи и подтверждение
  sendStart(STA_WAIT);
  datetime_t t;
  if (!rtc_get_datetime(&t))
  {
    lastError = ERR_DATETIME;
    return;
  }
  toDatetime(&sDate, &t);

  if (!rtc_set_datetime(&t))
  {
    lastError = ERR_DATETIME;
    return;
  }
  // send (STA_OK_CMD);
  sendStart(STA_OK_CMD);
}

/*******************************************************************************
 * Получить время                                                               *
 *******************************************************************************/
void cmd_get_time()
{
  sendStart(STA_WAIT);
  RTC_TimeTypeDef sTime = {0};

  datetime_t t;
  if (!rtc_get_datetime(&t))
  {
    lastError = ERR_DATETIME;
    return;
  }
  toRTC_Time(&t, &sTime);
  sendBin((BYTE *)&sTime, 3);
  sendBin((BYTE *)&sTime.SecondFraction, 1);
  sendBin((BYTE *)&sTime.SubSeconds, 1);
  sendByte(STA_OK_CMD);
  lastError = 0; // STA_OK_CMD;
}

/*******************************************************************************
 * Установить время                                                               *
 *******************************************************************************/
void cmd_set_time()
{
  RTC_TimeTypeDef sTime = {0};
  sTime.Hours = recvByte();
  sTime.Minutes = recvByte();
  sTime.Seconds = recvByte();
  sTime.SecondFraction = recvByte();
  sTime.SubSeconds = recvByte();
  datetime_t t;
  if (!rtc_get_datetime(&t))
  {
    lastError = ERR_DATETIME;
    return;
  }
  toDatetimeT(&sTime, &t);

  if (!rtc_set_datetime(&t))
  {
    lastError = ERR_DATETIME;
    return;
  }
  // send (STA_OK_CMD);
  sendStart(STA_OK_CMD);
}

/*******************************************************************************
 * Главная процедура                                                            *
 *******************************************************************************/

void error()
{
  for (;;)
  {
    LedOff();
    sleep_ms(100);
    LedOn();
    sleep_ms(100);
  }
}

bool lastLedVal;

struct
{
  uint8_t WRITE_MODE;
  uint8_t ARG_SELDISK;
  uint16_t ARG_TRACK;
  uint16_t ARG_SECTOR_128;
} a;
const uint16_t ARG_SEC_ON_TRK = 80;
uint16_t ARG_SECTOR_512;
uint8_t NEW_COUNT;
uint8_t NEW_DISK;
uint16_t NEW_TRACK;
uint16_t NEW_SECTOR;
int8_t BUFFER_DISK = -1;
uint16_t BUFFER_TRACK;
uint16_t BUFFER_SECTOR;
BYTE BUFFER[512];

bool BUFFER_CHANGED = false;

void cmd_bios_home()
{
  a.ARG_TRACK = 0;
}

void cmd_bios_sel_dsk()
{
  a.ARG_SELDISK = recvByte();
}

void cmd_bios_set_trk()
{
  a.ARG_TRACK = recvWord();
}

void cmd_bios_set_sect()
{
  a.ARG_SECTOR_128 = recvByte();
}

bool OpenDiskImage()
{
  if (BUFFER_DISK != a.ARG_SELDISK)
  {
    char fname[] = "CPM/A.KDI";
    fname[4] = 'A' + a.ARG_SELDISK;
    strcpy(buf, fname);
    if (fs_open() != 0)
      return false;
    BUFFER_DISK = a.ARG_SELDISK;
  }
  return true;
}

bool DiskSeek()
{
  // trk*1024*5*2/256
  DWORD off = BUFFER_TRACK * 1024 * 5 * 2 + BUFFER_SECTOR * 512;
  return fs_lseek(off, 0) == 0;
}

bool Write512()
{
  if (!OpenDiskImage() || !DiskSeek())
    return false;
  fs_wtotal = 512;
  // Запись данных
  do
  {
    MTX_ENTER();
    BYTE res = fs_write_start();
    MTX_EXIT();
    if (res)
      return false;
    MTX_ENTER();
    memcpy(fs_file_wbuf, BUFFER, 512);
    res = fs_write_end();
    MTX_EXIT();
    if (res)
      return false;
  } while (fs_wtotal);
  return true;
}

bool Read512()
{
  if (!OpenDiskImage() || !DiskSeek())
    return false;
  // Читаем блок
  MTX_ENTER();
  BYTE res = fs_read0(BUFFER, 512);
  MTX_EXIT();
  return res == 0;
}

bool SaveBuffer()
{
  BUFFER_CHANGED = false;
  return Write512();
}

#define BW_READ true
#define BW_WRITE false
#define BW_READ_512 true
#define BW_BLOCK_READ_512 false

bool BufferWork(bool bRead, bool C);

inline bool BiosRead()
{
  a.WRITE_MODE = 2;
  return BufferWork(BW_READ, BW_READ_512);
}

bool BufferWork(bool bRead, bool C)
{
  ARG_SECTOR_512 = a.ARG_SECTOR_128 >> 2;
  if (BUFFER_DISK != a.ARG_SELDISK || BUFFER_TRACK != a.ARG_TRACK || BUFFER_SECTOR != ARG_SECTOR_512)
  {
    if (BUFFER_DISK != -1 && BUFFER_CHANGED && !SaveBuffer())
      return false;
    BUFFER_CHANGED = false;
    if (!OpenDiskImage())
      return false;
    BUFFER_DISK = a.ARG_SELDISK;
    BUFFER_TRACK = a.ARG_TRACK;
    BUFFER_SECTOR = ARG_SECTOR_512;
    if (C && !Read512())
    {
      BUFFER_DISK = -1;
      return false;
    }
  }
  size_t addr = (a.ARG_SECTOR_128 & 3) * 128;
  BUFFER_CHANGED = true;
  sendByte(STA_OK_BLOCK);
  if (bRead)
  {
#if USE_DMA
    dma_send(BUFFER + addr, 128);
#else
    sendBin(BUFFER + addr, 128);
    sendByte(STA_WAIT);
#endif
  }
  else
  {
#if !USE_DMA
    recvStart();
    recvBin(BUFFER + addr, 128);
#else
    dma_receive(BUFFER + addr, 128);
#endif
  }
  if (a.WRITE_MODE != 1)
    return true;
  return SaveBuffer();
}

void cmd_bios_rd_rect()
{
  NEW_COUNT = 0;
  dma_receive(&a.ARG_SELDISK, 5);
  bool res = BiosRead();
  sendByte(res ? STA_OK_CMD : ERR_DISK_ERR);
}

bool CheckTrack()
{
  ++NEW_SECTOR;
  if (NEW_SECTOR == ARG_SEC_ON_TRK)
  {
    NEW_SECTOR = 0;
    ++NEW_TRACK;
  }
  return BufferWork(BW_WRITE, BW_BLOCK_READ_512);
}

inline bool ReadWrite512()
{
  NEW_COUNT = 0;
  return BufferWork(BW_WRITE, BW_READ_512);
}

bool BiosWrite()
{
  if (a.WRITE_MODE == 2)
  {
    // Если это запись в блок файловой системы, который до этого не
    // использовался, то взводится счетчик блокирующий чтение с дискеты
    // BIOS_WRITE_FIRST_NEW
    NEW_COUNT = 0xF;
    NEW_DISK = a.ARG_SELDISK;
    NEW_TRACK = a.ARG_TRACK;
    NEW_SECTOR = a.ARG_SECTOR_128;
    return CheckTrack();
  }
  if (NEW_COUNT == 0)
    return ReadWrite512();
  --NEW_COUNT;
  if (NEW_DISK != a.ARG_SELDISK || NEW_TRACK != a.ARG_TRACK || NEW_SECTOR != a.ARG_SECTOR_128)
    return ReadWrite512();
  return CheckTrack();
}

void cmd_bios_wr_rect()
{
  dma_receive(&a.WRITE_MODE, 6);
  bool res = BiosWrite();
  sendByte(res ? STA_OK_CMD : ERR_DISK_ERR);
}

enum
{
  CMD_BIOS_HOME = 10,
  CMD_BIOS_SEL_DSK,
  CMD_BIOS_SET_TRK,
  CMD_BIOS_SET_SECT,
  CMD_BIOS_RD_RECT,
  CMD_BIOS_WR_RECT,
};

BYTE __not_in_flash_func(RkSd_Loop())
{
  BYTE c;
  // while (1)
  {
    // Проверяем наличие карты
    sendStart(STA_START);
#if !USE_DMA
    sendByte(STA_WAIT);
#endif
    // if (fs_check ())
    //{
    //   sendByte (ERR_DISK_ERR);
    // }
    // else
    {
      sendByte(STA_OK_DISK);
      recvStart();
      c = recvByte();
      // Зажигаем светодиод
      LedOn();

      // Сбрасываем ошибку
      lastError = 0;

      //MTX_ENTER();
      // Принимаем аргументы
      switch (c)
      {
      case 0:
        cmd_boot();
        break;
      case 1:
        cmd_ver();
        break;
      case 2:
        cmd_exec();
        break;
      case 3:
        cmd_find();
        break;
      case 4:
        cmd_open();
        break;
      case 5:
        cmd_lseek();
        break;
      case 6:
        cmd_read();
        break;
      case 7:
        cmd_write();
        break;
      case 8:
        cmd_move();
        break;
      case 9:
        LedOff();
        return 0;
      case CMD_BIOS_HOME:
        cmd_bios_home();
        break;
      case CMD_BIOS_SEL_DSK:
        cmd_bios_sel_dsk();
        break;
      case CMD_BIOS_SET_TRK:
        cmd_bios_set_trk();
        break;
      case CMD_BIOS_SET_SECT:
        cmd_bios_set_sect();
        break;
      case CMD_BIOS_RD_RECT:
        cmd_bios_rd_rect();
        break;
      case CMD_BIOS_WR_RECT:
        cmd_bios_wr_rect();
        break;
      case 0x2A:
        cmd_get_date();
        break;
      case 0x2B:
        cmd_set_date();
        break;
      case 0x2C:
        cmd_get_time();
        break;
      case 0x2D:
        cmd_set_time();
        break;
      default:
        lastError = ERR_INVALID_COMMAND;
      }
      //MTX_EXIT();

      // Вывод ошибки
      if (lastError && c != STA_START)
      {
        // if (lastError!=ERR_FILE_EXISTS)
        //   panic("error");
        sendStart(lastError);
      }
    }

#if 0 // !USE_DMA
    // Порт работает на выход
    wait();
    DATA_OUT();
#endif

    // Гасим светодиод
    LedOff();
    return 1;
  }
}

// const uint dmaReadSm = 0;
const uint dmaWriteSm = 0;
const uint dmaRomSm = 2;
const uint dmaReadSm = 1;
const uint fifoReadSm = 0;
const uint fifoWrite2Sm = 1;
extern int res;

volatile uint8_t v55_buf[4] = {0, 0, 0, 0};

#if USE_DMA

static inline void WRITE_DATA(BYTE c)
{
  gpio_put_masked(GPIO_CD_MASK, ((uint32_t)c) << PIN_CD7);
}

static inline BYTE READ_DATA()
{
  return (gpio_get_all() & GPIO_CD_MASK) >> PIN_CD7;
}

#else // !USE_DMA
static inline void WRITE_DATA(BYTE c)
{
  uint32_t val;
  do
  {
    val = gpio_get_all();
  } while ((val & (nCS2_MASK | nRD_MASK)) != 0);
  gpio_set_dir_out_masked(GPIO_CD_MASK);
  gpio_put_masked(GPIO_CD_MASK, ((uint32_t)c) << PIN_CD7);
  do
  {
    val = gpio_get_all();
  } while ((val & (nCS2_MASK | nRD_MASK)) == 0);
  gpio_set_dir_in_masked(GPIO_CD_MASK);
}

static __force_inline uint16_t __not_in_flash_func(READ_ADDR)()
{
  return *(uint16_t *)&v55_buf[1]; // | (((uint32_t)v55_buf[2]) << 8);
}

bool bDir = false;

#endif

// void RkSd_main();
auto_init_mutex(sd_mutex);
recursive_mutex_t sd_mutex2;

extern volatile uint16_t addr;
extern volatile bool bStopRomEmu;

void __not_in_flash_func(EmulateRom)()
{
  while (!bStopRomEmu)
    ;
}

int res = 0;
void __not_in_flash_func(main_sd)()
{
  busy_wait_ms(500);
  multicore_fifo_pop_blocking();
  DATA_IN();
  LedOn();
  // Пауза, пока не стабилизируется питание
  // sleep_ms(300);
  // Запуск файловой системы
  if (fs_init())
    error();
  {
    // mutex_enter_blocking(&sd_mutex);
    MTX_ENTER();
    strcpy(buf, "boot/boot.rk");
    if (fs_open())
      error();
    if (fs_getfilesize())
      error();
    if (fs_tmp < 7)
      error();
    if (fs_tmp > 256)
      error();
    {
      WORD rom_size = (WORD)fs_tmp;
      if (fs_read0(rom, rom_size))
        error();

      /* delay_ms */ // sleep_ms(100);

      // Гасим светодиод
      LedOff();
#if 0
    //sendStart(0x45);
    //sendFlush();
#if 0
    for (int i=0; i<rom_size; ++i)
      rom[i] = (i%2 == 0) ? 0 : 0xFE;
#endif
    while (1)
    {
      dma_send(rom, rom_size);
      dma_receive(buf, rom_size);
      dma_send(buf, rom_size);
      //static int n = 0;
      //n = memcmp(buf, rom, rom_size);
    }
#endif
    }
    // mutex_exit(&sd_mutex);
    MTX_EXIT();
  }
  // while (true) ;
#if !USE_DMA
  EmulateRom();
#endif
  while (1)
  {
    if (!RkSd_Loop())
      return;
  }
}
