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

BYTE buf[512];
BYTE rom[128];
#define flash

/*******************************************************************************
 * Для удобства                                                                 *
 *******************************************************************************/

void recvBin(BYTE *d, WORD l)
{
  for (; l; --l)
  {
    *d++ = wrecv();
  }
}

void recvString()
{
  BYTE c;
  BYTE *p = buf;
  do
  {
    c = wrecv();
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

  while (readLength)
  {
    // Расчет длины блока (выравниваем чтение на сектор)
    if (fs_tell())
      return;
    readedLength = 512 - (fs_tmp % 512);
    if (readedLength > readLength)
      readedLength = readLength;

    // Уменьшаем счетчик
    readLength -= readedLength;

    // Читаем блок
    if (fs_read0(buf, readedLength))
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
      sendBin(buf, 2);
#ifndef USE_DMA
      sendByte (STA_WAIT);
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
    sendBin((BYTE *)&readedLength, 2);
#ifdef USE_DMA
    sendFlush();
    sleep_ms(10);
    dma_send(wptr, readedLength);
    //BYTE i=0;//
#else
    sendBin (wptr, readedLength);
    sendByte (STA_WAIT);
#endif
  }

  // Если все ОК
  if (!lastError)
    lastError = STA_OK_READ;
}

/*******************************************************************************
 * Версия команд контроллера                                                    *
 *******************************************************************************/

void cmd_ver()
{
  sendStart(1);

  // Версия + Производитель
  {
    flash char *ver = "V1.0 10-05-2014 ";
    sendBinf(ver, 16);
  }
#ifdef USE_DMA
  sendFlush();
#endif
}

/*******************************************************************************
 * BOOT / EXEC                                                                  *
 *******************************************************************************/

void cmd_boot_exec()
{
  // Файл по умолчанию
#ifndef USE_DMA
    const char *bootSdbiosRk = "boot/sdbios.rk";
#else
    const char *bootSdbiosRk = "boot/sdbiosd.rkl";
#endif
    if (buf[0] == 0)
      strcpy((char*) buf, /* (const char*) (nCS_GPIO_Port->IDR & nCS_Pin) ?  "boota/sdbios.rk" :  */bootSdbiosRk);

  // Открываем файл
  if (fs_open())
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
  sendStart (STA_WAIT);
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
  recvBin((BYTE *)&n, 2);

  // Режим передачи и подтверждение
  sendStart(STA_WAIT);
  if (lastError)
    return;

  // Открываем папку
  if (buf[0] != ':')
  {
    if (fs_opendir())
      return;
  }

  for (; n; --n)
  {
    /* Читаем очередной описатель */
    if (fs_readdir())
      return;

    /* Конец */
    if (FS_DIRENTRY[0] == 0)
    {
      lastError = STA_OK_CMD;
      return;
    }

    /* Сжимаем ответ для компьютера */
    memcpy(info.fname, FS_DIRENTRY + DIR_Name, 12);
    memcpy(&info.fsize, FS_DIRENTRY + DIR_FileSize, 4);
    memcpy(&info.ftimedate, FS_DIRENTRY + DIR_WrtTime, 4);
    // memcpy(memcpy(memcpy(info.fname, FS_DIRENTRY+DIR_Name, 12, FS_DIRENTRY+DIR_FileSize, 4), FS_DIRENTRY+DIR_WrtTime, 4);

    /* Отправляем */
    sendByte(STA_OK_ENTRY);
#ifndef USE_DMA
    sendBin ((BYTE*) &info, sizeof(info));
    sendByte (STA_WAIT);
#else
    sendFlush();
    dma_send((BYTE*) &info, sizeof(info));
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
  mode = wrecv();

  // Принимаем имя файла
  recvString();

  // Режим передачи и подтверждение
  sendStart(STA_WAIT);

  // Открываем/создаем файл/папку
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

  // Ок
  if (!lastError)
    lastError = STA_OK_CMD;
}

/*******************************************************************************
 * Переместить файл/папку                                                       *
 *******************************************************************************/

void cmd_move()
{
  recvString();
  sendStart(STA_WAIT);
  fs_openany();
  sendStart(STA_OK_WRITE);
  recvStart();
  recvString();
  sendStart(STA_WAIT);
  if (!lastError)
    fs_move0();
  if (!lastError)
    lastError = STA_OK_CMD;
}

/*******************************************************************************
 * Установить/прочитать указатель чтения                                        *
 *******************************************************************************/

void cmd_lseek()
{
  BYTE mode;
  DWORD off = 0;

  // Принимаем режим и смещение
  mode = wrecv();
  recvBin((BYTE *)&off, 4);

  // Режим передачи и подтверждение
  sendStart(STA_WAIT);

  // Размер файла
  if (mode == 100)
  {
    if (fs_getfilesize())
      return;
  }

  // Размер диска
  else if (mode == 101)
  {
    if (fs_gettotal())
      return;
  }

  // Свободное место на диске
  else if (mode == 102)
  {
    if (fs_getfree())
      return;
  }

  else
  {
    /* Устаналиваем смещение. fs_tmp сохраняется */
    if (fs_lseek(off, mode))
      return;
  }

  // Передаем результат
  sendByte(STA_OK_CMD);
  sendBin((BYTE *)&fs_tmp, 4);
  lastError = 0; // На всякий случай, результат уже передан
}

/*******************************************************************************
 * Прочитать из файла                                                           *
 *******************************************************************************/

void cmd_read()
{
  DWORD s;

  // Длина
  recvBin((BYTE *)&readLength, 2);

  // Режим передачи и подтверждение
  sendStart(STA_WAIT);

  // Ограничиваем длину длиной файла
  if (fs_getfilesize())
    return;
  s = fs_tmp;
  if (fs_tell())
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
  recvBin((BYTE *)&fs_wtotal, 2);

  // Ответ
  sendStart(STA_WAIT);

  // Конец файла
  if (fs_wtotal == 0)
  {
    fs_write_eof();
    lastError = STA_OK_CMD;
    return;
  }

  // Запись данных
  do
  {
    if (fs_write_start())
      return;

    // Принимаем от компьютера блок данных
    sendByte(STA_OK_WRITE);
    sendBin((BYTE *)&fs_file_wlen, 2);
#ifndef USE_DMA
    recvStart();
    recvBin(fs_file_wbuf, fs_file_wlen);
#else
    recvStartNoDma();
    dma_receive(fs_file_wbuf, fs_file_wlen);
#endif
    sendStart(STA_WAIT);

    if (fs_write_end())
      return;
  } while (fs_wtotal);

  lastError = STA_OK_CMD;
}

typedef struct
{
  uint8_t Hours;            /*!< Specifies the RTC Time Hour.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 12 if the RTC_HourFormat_12 is selected
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 23 if the RTC_HourFormat_24 is selected */

  uint8_t Minutes;          /*!< Specifies the RTC Time Minutes.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t Seconds;          /*!< Specifies the RTC Time Seconds.
                                 This parameter must be a number between Min_Data = 0 and Max_Data = 59 */

  uint8_t TimeFormat;       /*!< Specifies the RTC AM/PM Time.
                                 This parameter can be a value of @ref RTC_AM_PM_Definitions */

  uint32_t SubSeconds;      /*!< Specifies the RTC_SSR RTC Sub Second register content.
                                 This parameter corresponds to a time unit range between [0-1] Second
                                 with [1 Sec / SecondFraction +1] granularity */

  uint32_t SecondFraction;  /*!< Specifies the range or granularity of Sub Second register content
                                 corresponding to Synchronous prescaler factor value (PREDIV_S)
                                 This parameter corresponds to a time unit range between [0-1] Second
                                 with [1 Sec / SecondFraction +1] granularity.
                                 This field will be used only by HAL_RTC_GetTime function */

  uint32_t DayLightSaving;  /*!< This interface is deprecated. To manage Daylight
                                 Saving Time, please use HAL_RTC_DST_xxx functions */

  uint32_t StoreOperation;  /*!< This interface is deprecated. To manage Daylight
                                 Saving Time, please use HAL_RTC_DST_xxx functions */
} RTC_TimeTypeDef;



//extern RTC_HandleTypeDef hrtc;

static const uint8_t list_mth[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

uint8_t Calendar_GetDayWeek (RTC_DateTypeDef thisDate)
{
	uint8_t ret = 0;
    if(thisDate.Month < 3){
    	thisDate.Year -= 1;
    }
    ret = (uint8_t)((thisDate.Year + (thisDate.Year/4) - (thisDate.Year/100) + (thisDate.Year/400) + list_mth[thisDate.Month-1] + thisDate.Date) % 7);
	return (ret);
}

void toDatetime(const RTC_DateTypeDef *rt, datetime_t * t)
{
  if (!t || !rt)
    return;
  t->day = rt->Date;
  t->month = rt->Month;
  t->dotw = rt->WeekDay;
  t->year = rt->Year + 20000;
}

void toDatetimeT(const RTC_TimeTypeDef *rt, datetime_t * t)
{
  if (!t || !rt)
    return;
  t->sec = rt->Seconds;
  t->min = rt->Minutes;
  t->hour = rt->Hours;
}

void toRTC_Date(const  datetime_t * t, RTC_DateTypeDef *rt)
{
  if (!t || !rt)
    return;
  rt->Date    = t->day  ;
  rt->Month   = t->month;
  rt->WeekDay = t->dotw ;
  rt->Year    = t->year - 2000 ;
}

void toRTC_Time(const  datetime_t * t, RTC_TimeTypeDef *rt)
{
  if (!t || !rt)
    return;
  rt->Hours   = t->hour ;
  rt->Minutes = t->min;
  rt->Seconds = t->sec;
  rt->SecondFraction = 255 ;
  rt->SubSeconds = 0;
}
/*******************************************************************************
 * Получить дату                                                                *
 *******************************************************************************/
void cmd_get_date()
{
  sendStart(STA_WAIT);
  RTC_DateTypeDef sDate={0};
  datetime_t t;
  if (!rtc_get_datetime(&t))
  {
    lastError = ERR_DATETIME;
    return;

  }
  toRTC_Date(&t, &sDate);
  sendBin((BYTE*)&sDate, sizeof(sDate));
  sendByte(STA_OK_CMD);
  lastError = 0;//STA_OK_CMD;
}


/*******************************************************************************
 * Установить дату                                                              *
 *******************************************************************************/
void cmd_set_date()
{
  RTC_DateTypeDef sDate={0};
  //recvStart();
  sDate.WeekDay = wrecv();
  sDate.Month = wrecv();
  sDate.Date = wrecv();
  sDate.Year = wrecv();
  sDate.WeekDay = Calendar_GetDayWeek(sDate);

  // Режим передачи и подтверждение
  sendStart (STA_WAIT);
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
  //send (STA_OK_CMD);
  lastError = STA_OK_CMD;
}

/*******************************************************************************
 * Получить время                                                               *
 *******************************************************************************/
void cmd_get_time()
{
  sendStart(STA_WAIT);
  RTC_TimeTypeDef sTime={0};

  datetime_t t;
  if (!rtc_get_datetime(&t))
  {
    lastError = ERR_DATETIME;
    return;

  }
  toRTC_Time(&t, &sTime);
  sendBin((BYTE*)&sTime, 3);
  sendBin((BYTE*)&sTime.SecondFraction,1);
  sendBin((BYTE*)&sTime.SubSeconds,1);
  sendByte(STA_OK_CMD);
  lastError = 0;//STA_OK_CMD;
}

/*******************************************************************************
 * Установить время                                                               *
 *******************************************************************************/
void cmd_set_time()
{
  RTC_TimeTypeDef sTime={0};
  sTime.Hours = wrecv();
  sTime.Minutes = wrecv();
  sTime.Seconds = wrecv();
  sTime.SecondFraction = wrecv();
  sTime.SubSeconds = wrecv();
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
  //send (STA_OK_CMD);
  lastError = STA_OK_CMD;
}


/*******************************************************************************
 * Главная процедура                                                            *
 *******************************************************************************/
void LedOn();
void LedOff();

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

void LedOff()
{
  // Гасим светодиод
  gpio_put(LED, 0);
}

void LedOn()
{
  // Зажигаем светодиод
  gpio_put(LED, 1);
}


BYTE RkSd_Loop()
{
  BYTE c;
  // while (1)
  {

    // Проверяем наличие карты
    sendStart(STA_START);
#ifndef USE_DMA
    sendByte (STA_WAIT);
#endif
    //if (fs_check ())
    //{
    //  sendByte (ERR_DISK_ERR);
    //}
    //else
    {
      sendByte(STA_OK_DISK);
      recvStart();
      c = wrecv();
      // Зажигаем светодиод
      LedOn();

      // Сбрасываем ошибку
      lastError = 0;

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
#if 1
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
#endif
      default:
        lastError = ERR_INVALID_COMMAND;
      }

      // Вывод ошибки
      if (lastError && c != STA_START) 
        sendStart(lastError);
#ifdef USE_DMA
      sendFlush();
#endif
    }

#ifndef USE_DMA
    // Порт работает на выход
    wait();
    DATA_OUT();
#endif

    // Гасим светодиод
    LedOff();
    return 1;
  }
}

#ifdef USE_DMA

static inline void WRITE_DATA(BYTE c)
{
  gpio_put_masked(GPIO_CD_MASK, ((uint32_t)c) << GPIO_CD7);
}
//const uint dmaReadSm = 0;
const uint dmaWriteSm = 0;
const uint dmaReadSm = 1;
const uint fifoReadSm = 0;
const uint fifoWrite2Sm = 1;
extern int res;
//extern const uint fifoWriteSm;
extern const uint fifoReadSm;

void __not_in_flash_func(dma_send)(BYTE *ptr, WORD len)
{
#if 1
  gpio_init(DRQ);
  gpio_set_dir(DRQ, GPIO_OUT);
  gpio_put(DRQ, 1);
  pio_gpio_init(FIFO_PIO, DIR);
  do
  {
    pio_sm_put_blocking(FIFO_PIO, dmaReadSm, *ptr++ | (0xFF << 8));
  } while (--len);
  while (!pio_sm_is_tx_fifo_empty(FIFO_PIO, dmaReadSm)) ;
  //sleep_ms(1);
  gpio_put(DRQ, 0);
  //while (gpio_get(DIR)==1) ;
  //while (gpio_get(DIR)==0) ;
#else
  gpio_init(DRQ);
  gpio_put(DRQ, 1);
  gpio_set_dir(DRQ, GPIO_OUT);
  uint32_t ints = save_and_disable_interrupts();
  do
  {
    DATA_IN();
    while (gpio_get_all() & (nIOR_MASK/*  | nDACK_MASK */))
      ;
    DATA_OUT();
    WRITE_DATA(*ptr++); // PORTD = *ptr++;
    while (gpio_get(nIOR) == 0)
      ;
  } while (--len);
  gpio_put(DRQ, 0);
  restore_interrupts(ints);
#endif
}

static inline BYTE READ_DATA()
{
  return (gpio_get_all() & GPIO_CD_MASK) >> GPIO_CD7;
}

void __not_in_flash_func(dma_receive)(BYTE *ptr, WORD len)
{
#if 1
  // while (!pio_sm_is_rx_fifo_empty(DMA_PIO, dmaWriteSm))
  //   pio_sm_get(DMA_PIO, dmaWriteSm);

  //pio_sm_clear_fifos(DMA_PIO, dmaWriteSm);
  //pio_sm_restart(DMA_PIO, dmaWriteSm);
  uint mask = DRQ_MASK | DIR_MASK;
  gpio_init_mask(mask);
  gpio_put_masked(mask, mask);
  gpio_set_dir_out_masked(mask);
  //pio_sm_set_enabled(DMA_PIO, dmaWriteSm, true);
  //pio_sm_get_blocking(DMA_PIO, dmaWriteSm);
  do
  {
    *ptr++ = pio_sm_get_blocking(DMA_PIO, dmaWriteSm) & 0xFF;
  } while (--len);
  gpio_put(DRQ, 0);
  //pio_sm_set_enabled(DMA_PIO, dmaWriteSm, false);
  #else
  DATA_IN();
  gpio_put(DRQ, 1);
  uint32_t ints = save_and_disable_interrupts();
  do
  {
    while (gpio_get_all() & (nIOW_MASK | nDACK_MASK))
      ;
    while (gpio_get(nIOW) == 0)
      ;
    *ptr++ = READ_DATA();
  } while (--len);
  gpio_put(DRQ, 0);
  restore_interrupts(ints);
#endif
}
#endif
int res = 0;
void main_sd()
{
  DATA_IN();
  LedOn();
  // Пауза, пока не стабилизируется питание
  /* delay_ms */ //sleep_ms(500);

  // Запуск файловой системы
  if (fs_init())
    error();
  multicore_fifo_push_blocking(0);
  while (true) ;
  strcpy(buf, "boot/boot.rk");
  if (fs_open())
    error();
  if (fs_getfilesize())
    error();
  if (fs_tmp < 7)
    error();
  if (fs_tmp > 128)
    error();
  {
    WORD rom_size = (WORD)fs_tmp;
    if (fs_read0(rom, rom_size))
      error();

    /* delay_ms */ //sleep_ms(100);

    // Гасим светодиод
    LedOff();
#if 0
    //sendStart(0x45);
    //sendFlush();
    dma_send(rom, rom_size);
    dma_receive(buf, rom_size);
    dma_send(buf, rom_size);
    //static int n = 0;
    //n = memcmp(buf, rom, rom_size);
    while (1) ;
#endif
  }
  //while (true) ;
  while (1)
  {
    if (!RkSd_Loop())
      return;
  }
}
