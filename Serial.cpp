/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "Serial.h"
#include "hardware/ticks.h"
#include "pico/time.h"
#include "tusb.h"

// using namespace arduino;

// Public Methods //////////////////////////////////////////////////////////////

/* default implementation: may be overridden */
size_t Print::write(const uint8_t *buffer, size_t size)
{
  size_t n = 0;
  while (size--)
  {
    if (write(*buffer++))
      n++;
    else
      break;
  }
  return n;
}

size_t Print::print(const String &s)
{
  return write(s.c_str(), s.length());
}

size_t Print::print(const char *str)
{
  return write(str);
}

size_t Print::print(char c)
{
  return write(c);
}

size_t Print::print(unsigned char b, int base)
{
  return print((unsigned long)b, base);
}

size_t Print::print(int n, int base)
{
  return print((long)n, base);
}

size_t Print::print(unsigned int n, int base)
{
  return print((unsigned long)n, base);
}

size_t Print::print(long n, int base)
{
  if (base == 0)
  {
    return write(n);
  }
  else if (base == 10)
  {
    if (n < 0)
    {
      int t = print('-');
      n = -n;
      return printNumber(n, 10) + t;
    }
    return printNumber(n, 10);
  }
  else
  {
    return printNumber(n, base);
  }
}

size_t Print::print(unsigned long n, int base)
{
  if (base == 0)
    return write(n);
  else
    return printNumber(n, base);
}

size_t Print::print(long long n, int base)
{
  if (base == 0)
  {
    return write(n);
  }
  else if (base == 10)
  {
    if (n < 0)
    {
      int t = print('-');
      n = -n;
      return printULLNumber(n, 10) + t;
    }
    return printULLNumber(n, 10);
  }
  else
  {
    return printULLNumber(n, base);
  }
}

size_t Print::print(unsigned long long n, int base)
{
  if (base == 0)
    return write(n);
  else
    return printULLNumber(n, base);
}

size_t Print::print(double n, int digits)
{
  return printFloat(n, digits);
}

size_t Print::println(void)
{
  return write("\r\n");
}

size_t Print::println(const String &s)
{
  size_t n = print(s);
  n += println();
  return n;
}

size_t Print::println(const char *c)
{
  size_t n = print(c);
  n += println();
  return n;
}

size_t Print::println(char c)
{
  size_t n = print(c);
  n += println();
  return n;
}

size_t Print::println(unsigned char b, int base)
{
  size_t n = print(b, base);
  n += println();
  return n;
}

size_t Print::println(int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(unsigned int num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(long num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(unsigned long num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(long long num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(unsigned long long num, int base)
{
  size_t n = print(num, base);
  n += println();
  return n;
}

size_t Print::println(double num, int digits)
{
  size_t n = print(num, digits);
  n += println();
  return n;
}

size_t Print::printf(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  char temp[64];
  char *buffer = temp;
  size_t len = vsnprintf(temp, sizeof(temp), format, arg);
  va_end(arg);
  if (len > sizeof(temp) - 1)
  {
    buffer = new char[len + 1];
    if (!buffer)
    {
      return 0;
    }
    va_start(arg, format);
    vsnprintf(buffer, len + 1, format, arg);
    va_end(arg);
  }
  len = write((const uint8_t *)buffer, len);
  if (buffer != temp)
  {
    delete[] buffer;
  }
  return len;
}

// TODO - must be better way than cut-n-paste!
size_t Print::printf_P(const char *format, ...)
{
  va_list arg;
  va_start(arg, format);
  char temp[64];
  char *buffer = temp;
  size_t len = vsnprintf(temp, sizeof(temp), format, arg);
  va_end(arg);
  if (len > sizeof(temp) - 1)
  {
    buffer = new char[len + 1];
    if (!buffer)
    {
      return 0;
    }
    va_start(arg, format);
    vsnprintf(buffer, len + 1, format, arg);
    va_end(arg);
  }
  len = write((const uint8_t *)buffer, len);
  if (buffer != temp)
  {
    delete[] buffer;
  }
  return len;
}

// Private Methods /////////////////////////////////////////////////////////////

size_t Print::printNumber(unsigned long n, uint8_t base)
{
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *str = &buf[sizeof(buf) - 1];

  *str = '\0';

  // prevent crash if called with base == 1
  if (base < 2)
    base = 10;

  do
  {
    char c = n % base;
    n /= base;

    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (n);

  return write(str);
}

// REFERENCE IMPLEMENTATION FOR ULL
// size_t Print::printULLNumber(unsigned long long n, uint8_t base)
// {
// // if limited to base 10 and 16 the bufsize can be smaller
// char buf[65];
// char *str = &buf[64];

// *str = '\0';

// // prevent crash if called with base == 1
// if (base < 2) base = 10;

// do {
// unsigned long long t = n / base;
// char c = n - t * base;  // faster than c = n%base;
// n = t;
// *--str = c < 10 ? c + '0' : c + 'A' - 10;
// } while(n);

// return write(str);
// }

// FAST IMPLEMENTATION FOR ULL
size_t Print::printULLNumber(unsigned long long n64, uint8_t base)
{
  // if limited to base 10 and 16 the bufsize can be 20
  char buf[64];
  uint8_t i = 0;
  uint8_t innerLoops = 0;

  // Special case workaround https://github.com/arduino/ArduinoCore-API/issues/178
  if (n64 == 0)
  {
    write('0');
    return 1;
  }

  // prevent crash if called with base == 1
  if (base < 2)
    base = 10;

  // process chunks that fit in "16 bit math".
  uint16_t top = 0xFFFF / base;
  uint16_t th16 = 1;
  while (th16 < top)
  {
    th16 *= base;
    innerLoops++;
  }

  while (n64 > th16)
  {
    // 64 bit math part
    uint64_t q = n64 / th16;
    uint16_t r = n64 - q * th16;
    n64 = q;

    // 16 bit math loop to do remainder. (note buffer is filled reverse)
    for (uint8_t j = 0; j < innerLoops; j++)
    {
      uint16_t qq = r / base;
      buf[i++] = r - qq * base;
      r = qq;
    }
  }

  uint16_t n16 = n64;
  while (n16 > 0)
  {
    uint16_t qq = n16 / base;
    buf[i++] = n16 - qq * base;
    n16 = qq;
  }

  size_t bytes = i;
  for (; i > 0; i--)
    write((char)(buf[i - 1] < 10 ? '0' + buf[i - 1] : 'A' + buf[i - 1] - 10));

  return bytes;
}

size_t Print::printFloat(double number, int digits)
{
  if (digits < 0)
    digits = 2;

  size_t n = 0;

  if (isnan(number))
    return print("nan");
  if (isinf(number))
    return print("inf");
  if (number > 4294967040.0)
    return print("ovf"); // constant determined empirically
  if (number < -4294967040.0)
    return print("ovf"); // constant determined empirically

  // Handle negative numbers
  if (number < 0.0)
  {
    n += print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  n += print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
  {
    n += print(".");
  }

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    unsigned int toPrint = (unsigned int)remainder;
    n += print(toPrint);
    remainder -= toPrint;
  }

  return n;
}

/*
 Stream.cpp - adds parsing methods to Stream class
 Copyright (c) 2008 David A. Mellis.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

 Created July 2011
 parsing functions based on TextFinder library by Michael Margolis

 findMulti/findUntil routines written by Jim Leonard/Xuth
 */

#define PARSE_TIMEOUT 1000 // default number of milli-seconds to wait

// using namespace arduino;
uint32_t millis()
{
  return to_ms_since_boot(get_absolute_time());
}

// private method to read stream with timeout
int Stream::timedRead()
{
  int c;
  _startMillis = millis();
  do
  {
    c = read();
    if (c >= 0)
      return c;
  } while (millis() - _startMillis < _timeout);
  return -1; // -1 indicates timeout
}

// private method to peek stream with timeout
int Stream::timedPeek()
{
  int c;
  _startMillis = millis();
  do
  {
    c = peek();
    if (c >= 0)
      return c;
  } while (millis() - _startMillis < _timeout);
  return -1; // -1 indicates timeout
}

// returns peek of the next digit in the stream or -1 if timeout
// discards non-numeric characters
int Stream::peekNextDigit(LookaheadMode lookahead, bool detectDecimal)
{
  int c;
  while (1)
  {
    c = timedPeek();

    if (c < 0 ||
        c == '-' ||
        (c >= '0' && c <= '9') ||
        (detectDecimal && c == '.'))
      return c;

    switch (lookahead)
    {
    case SKIP_NONE:
      return -1; // Fail code.
    case SKIP_WHITESPACE:
      switch (c)
      {
      case ' ':
      case '\t':
      case '\r':
      case '\n':
        break;
      default:
        return -1; // Fail code.
      }
    case SKIP_ALL:
      break;
    }
    read(); // discard non-numeric
  }
}

// Public Methods
//////////////////////////////////////////////////////////////

void Stream::setTimeout(unsigned long timeout) // sets the maximum number of milliseconds to wait
{
  _timeout = timeout;
}

// find returns true if the target string is found
bool Stream::find(const char *target)
{
  return findUntil(target, strlen(target), NULL, 0);
}

// reads data from the stream until the target string of given length is found
// returns true if target string is found, false if timed out
bool Stream::find(const char *target, size_t length)
{
  return findUntil(target, length, NULL, 0);
}

// as find but search ends if the terminator string is found
bool Stream::findUntil(const char *target, const char *terminator)
{
  return findUntil(target, strlen(target), terminator, strlen(terminator));
}

// reads data from the stream until the target string of the given length is found
// search terminated if the terminator string is found
// returns true if target string is found, false if terminated or timed out
bool Stream::findUntil(const char *target, size_t targetLen, const char *terminator, size_t termLen)
{
  if (terminator == NULL)
  {
    MultiTarget t[1] = {{target, targetLen, 0}};
    return findMulti(t, 1) == 0;
  }
  else
  {
    MultiTarget t[2] = {{target, targetLen, 0}, {terminator, termLen, 0}};
    return findMulti(t, 2) == 0;
  }
}

// returns the first valid (long) integer value from the current position.
// lookahead determines how parseInt looks ahead in the stream.
// See LookaheadMode enumeration at the top of the file.
// Lookahead is terminated by the first character that is not a valid part of an integer.
// Once parsing commences, 'ignore' will be skipped in the stream.
long Stream::parseInt(LookaheadMode lookahead, char ignore)
{
  bool isNegative = false;
  long value = 0;
  int c;

  c = peekNextDigit(lookahead, false);
  // ignore non numeric leading characters
  if (c < 0)
    return 0; // zero returned if timeout

  do
  {
    if ((char)c == ignore)
      ; // ignore this character
    else if (c == '-')
      isNegative = true;
    else if (c >= '0' && c <= '9') // is c a digit?
      value = value * 10 + c - '0';
    read(); // consume the character we got with peek
    c = timedPeek();
  } while ((c >= '0' && c <= '9') || (char)c == ignore);

  if (isNegative)
    value = -value;
  return value;
}

// as parseInt but returns a floating point value
float Stream::parseFloat(LookaheadMode lookahead, char ignore)
{
  bool isNegative = false;
  bool isFraction = false;
  double value = 0.0;
  int c;
  double fraction = 1.0;

  c = peekNextDigit(lookahead, true);
  // ignore non numeric leading characters
  if (c < 0)
    return 0; // zero returned if timeout

  do
  {
    if ((char)c == ignore)
      ; // ignore
    else if (c == '-')
      isNegative = true;
    else if (c == '.')
      isFraction = true;
    else if (c >= '0' && c <= '9')
    { // is c a digit?
      if (isFraction)
      {
        fraction *= 0.1;
        value = value + fraction * (c - '0');
      }
      else
      {
        value = value * 10 + c - '0';
      }
    }
    read(); // consume the character we got with peek
    c = timedPeek();
  } while ((c >= '0' && c <= '9') || (c == '.' && !isFraction) || (char)c == ignore);

  if (isNegative)
    value = -value;

  return value;
}

// read characters from stream into buffer
// terminates if length characters have been read, or timeout (see setTimeout)
// returns the number of characters placed in the buffer
// the buffer is NOT null terminated.
//
size_t Stream::readBytes(char *buffer, size_t length)
{
  size_t count = 0;
  while (count < length)
  {
    int c = timedRead();
    if (c < 0)
      break;
    *buffer++ = (char)c;
    count++;
  }
  return count;
}

// as readBytes with terminator character
// terminates if length characters have been read, timeout, or if the terminator character  detected
// returns the number of characters placed in the buffer (0 means no valid data found)

size_t Stream::readBytesUntil(char terminator, char *buffer, size_t length)
{
  size_t index = 0;
  while (index < length)
  {
    int c = timedRead();
    if (c < 0 || (char)c == terminator)
      break;
    *buffer++ = (char)c;
    index++;
  }
  return index; // return number of characters, not including null terminator
}

String Stream::readString()
{
  String ret;
  int c = timedRead();
  while (c >= 0)
  {
    ret += (char)c;
    c = timedRead();
  }
  return ret;
}

String Stream::readStringUntil(char terminator)
{
  String ret;
  int c = timedRead();
  while (c >= 0 && (char)c != terminator)
  {
    ret += (char)c;
    c = timedRead();
  }
  return ret;
}

int Stream::findMulti(struct Stream::MultiTarget *targets, int tCount)
{
  // any zero length target string automatically matches and would make
  // a mess of the rest of the algorithm.
  for (struct MultiTarget *t = targets; t < targets + tCount; ++t)
  {
    if (t->len <= 0)
      return t - targets;
  }

  while (1)
  {
    int c = timedRead();
    if (c < 0)
      return -1;

    for (struct MultiTarget *t = targets; t < targets + tCount; ++t)
    {
      // the simple case is if we match, deal with that first.
      if ((char)c == t->str[t->index])
      {
        if (++t->index == t->len)
          return t - targets;
        else
          continue;
      }

      // if not we need to walk back and see if we could have matched further
      // down the stream (ie '1112' doesn't match the first position in '11112'
      // but it will match the second position so we can't just reset the current
      // index to 0 when we find a mismatch.
      if (t->index == 0)
        continue;

      int origIndex = t->index;
      do
      {
        --t->index;
        // first check if current char works against the new current index
        if ((char)c != t->str[t->index])
          continue;

        // if it's the only char then we're good, nothing more to check
        if (t->index == 0)
        {
          t->index++;
          break;
        }

        // otherwise we need to check the rest of the found string
        int diff = origIndex - t->index;
        size_t i;
        for (i = 0; i < t->index; ++i)
        {
          if (t->str[i] != t->str[i + diff])
            break;
        }

        // if we successfully got through the previous loop then our current
        // index is good.
        if (i == t->index)
        {
          t->index++;
          break;
        }

        // otherwise we just try the next index
      } while (t->index);
    }
  }
  // unreachable
  return -1;
}

// #include <tusb.h>
#include <pico/time.h>
#include <pico/binary_info.h>
#include <pico/bootrom.h>
#include <hardware/irq.h>
#include <pico/mutex.h>
#include <hardware/watchdog.h>
// #include <pico/unique_id.h>
#include <hardware/resets.h>

#ifndef DISABLE_USB_SERIAL
// Ensure we are installed in the USB chain
void __USBInstallSerial() { /* noop */ }
#endif

// SerialEvent functions are weak, so when the user doesn't define them,
// the linker just sets their address to 0 (which is checked below).
// The Serialx_available is just a wrapper around Serialx.available(),
// but we can refer to it weakly so we don't pull in the entire
// HardwareSerial instance if the user doesn't also refer to it.
extern void serialEvent() __attribute__((weak));

extern mutex_t __usb_mutex;
enum
{
  DebugEnable = 1
};
typedef int32_t BaseType_t;

class CoreMutex
{
public:
  CoreMutex(mutex_t *mutex, uint8_t option = DebugEnable);
  ~CoreMutex();

  operator bool()
  {
    return _acquired;
  }

private:
  mutex_t *_mutex;
  bool _acquired;
  uint8_t _option;
  BaseType_t _pxHigherPriorityTaskWoken;
};

CoreMutex::CoreMutex(mutex_t *mutex, uint8_t option)
{
  _mutex = mutex;
  _acquired = false;
  _option = option;
  _pxHigherPriorityTaskWoken = 0; // pdFALSE
  {
    uint32_t owner;
    if (!mutex_try_enter(_mutex, &owner))
    {
      if (owner == get_core_num())
      { // Deadlock!
        if (_option & DebugEnable)
        {
          // DEBUGCORE("CoreMutex - Deadlock detected!\n");
        }
        return;
      }
      mutex_enter_blocking(_mutex);
    }
  }
  _acquired = true;
}

CoreMutex::~CoreMutex()
{
  if (_acquired)
  {
    mutex_exit(_mutex);
  }
}

extern mutex_t __usb_mutex;
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT 0
#endif

void SerialUSB::begin(unsigned long baud)
{
  (void)baud; // ignored

  if (_running)
  {
    return;
  }
  // init device stack on configured roothub port
  tud_init(BOARD_TUD_RHPORT);

  _running = true;
}

void SerialUSB::end()
{
  // TODO
}

int SerialUSB::peek()
{
  CoreMutex m(&__usb_mutex, false);
  if (!_running || !m)
  {
    return 0;
  }

  uint8_t c;
  tud_task();
  return tud_cdc_peek(&c) ? (int)c : -1;
}

int SerialUSB::read()
{
  CoreMutex m(&__usb_mutex, false);
  if (!_running || !m)
  {
    return -1;
  }

  tud_task();
  if (tud_cdc_available())
  {
    return tud_cdc_read_char();
  }
  return -1;
}

int SerialUSB::available()
{
  CoreMutex m(&__usb_mutex, false);
  if (!_running || !m)
  {
    return 0;
  }

  tud_task();
  return tud_cdc_available();
}

int SerialUSB::availableForWrite()
{
  CoreMutex m(&__usb_mutex, false);
  if (!_running || !m)
  {
    return 0;
  }

  tud_task();
  return tud_cdc_write_available();
}

void SerialUSB::flush()
{
  CoreMutex m(&__usb_mutex, false);
  if (!_running || !m)
  {
    return;
  }

  tud_cdc_write_flush();
  tud_task();
}

size_t SerialUSB::write(uint8_t c)
{
  return write(&c, 1);
}

size_t SerialUSB::write(const uint8_t *buf, size_t length)
{
  CoreMutex m(&__usb_mutex, false);
  if (!_running || !m)
  {
    return 0;
  }

  static uint64_t last_avail_time;
  int written = 0;
  if (tud_cdc_connected() || _ignoreFlowControl)
  {
    for (size_t i = 0; i < length;)
    {
      int n = length - i;
      int avail = tud_cdc_write_available();
      if (n > avail)
      {
        n = avail;
      }
      if (n)
      {
        int n2 = tud_cdc_write(buf + i, n);
        tud_task();
        tud_cdc_write_flush();
        i += n2;
        written += n2;
        last_avail_time = time_us_64();
      }
      else
      {
        tud_task();
        tud_cdc_write_flush();
        if (!tud_cdc_connected() ||
            (!tud_cdc_write_available() && time_us_64() > last_avail_time + 1'000'000 /* 1 second */))
        {
          break;
        }
      }
    }
  }
  else
  {
    // reset our timeout
    last_avail_time = 0;
  }
  tud_task();
  return written;
}

SerialUSB::operator bool()
{
  CoreMutex m(&__usb_mutex, false);
  if (!_running || !m)
  {
    return false;
  }

  tud_task();
  return tud_cdc_connected();
}

void SerialUSB::ignoreFlowControl(bool ignore)
{
  _ignoreFlowControl = ignore;
}

static bool _dtr = false;
static bool _rts = false;
static int _bps = 115200;
static bool _rebooting = false;
const bool __isFreeRTOS = false;
static void CheckSerialReset()
{
  if (!_rebooting && (_bps == 1200) && (!_dtr))
  {
    if (__isFreeRTOS)
    {
      //__freertos_idle_other_core();
    }
    _rebooting = true;
    // Disable NVIC IRQ, so that we don't get bothered anymore
    irq_set_enabled(USBCTRL_IRQ, false);
    // Reset the whole USB hardware block
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block(RESETS_RESET_USBCTRL_BITS);
    // Delay a bit, so the PC can figure out that we have disconnected.
    busy_wait_ms(3);
    reset_usb_boot(0, 0);
    while (1)
      ; // WDT will fire here
  }
}

bool SerialUSB::dtr()
{
  return _dtr;
}

bool SerialUSB::rts()
{
  return _rts;
}

extern "C" void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
  (void)itf;
  _dtr = dtr ? true : false;
  _rts = rts ? true : false;
  CheckSerialReset();
}

#if 1
extern "C" void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *p_line_coding)
{
  (void)itf;
  _bps = p_line_coding->bit_rate;
  CheckSerialReset();
}
#endif

SerialUSB serial;

namespace arduino {
void serialEventRun(void)
 {
    if (serialEvent && serial.available())
    {
        serialEvent();
    }
 }
}

