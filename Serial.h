#ifndef SERIAL_H
#define SERIAL_H

#include <string>
using String=std::string;
#include <cstring>
#include <cstdint>

#define DEC 10
#define HEX 16

class Print
{
  private:
    int write_error;
    size_t printNumber(unsigned long, uint8_t);
    size_t printULLNumber(unsigned long long, uint8_t);
    size_t printFloat(double, int);
  protected:
    void setWriteError(int err = 1) { write_error = err; }
  public:
    Print() : write_error(0) {}

    int getWriteError() { return write_error; }
    void clearWriteError() { setWriteError(0); }

    virtual size_t write(uint8_t) = 0;
    size_t write(const char *str) {
      if (str == NULL) return 0;
      return write((const uint8_t *)str, strlen(str));
    }
    virtual size_t write(const uint8_t *buffer, size_t size);
    size_t write(const char *buffer, size_t size) {
      return write((const uint8_t *)buffer, size);
    }

    // default to zero, meaning "a single write may block"
    // should be overriden by subclasses with buffering
    virtual int availableForWrite() { return 0; }

    //size_t print(const __FlashStringHelper *);
    size_t print(const String &);
    size_t print(const char*);
    size_t print(char);
    size_t print(unsigned char, int = DEC);
    size_t print(int, int = DEC);
    size_t print(unsigned int, int = DEC);
    size_t print(long, int = DEC);
    size_t print(unsigned long, int = DEC);
    size_t print(long long, int = DEC);
    size_t print(unsigned long long, int = DEC);
    size_t print(double, int = 2);
    //size_t print(const Printable&);

    //size_t println(const __FlashStringHelper *);
    size_t println(const String &s);
    size_t println(const char*);
    size_t println(char);
    size_t println(unsigned char, int = DEC);
    size_t println(int, int = DEC);
    size_t println(unsigned int, int = DEC);
    size_t println(long, int = DEC);
    size_t println(unsigned long, int = DEC);
    size_t println(long long, int = DEC);
    size_t println(unsigned long long, int = DEC);
    size_t println(double, int = 2);
    //size_t println(const Printable&);
    size_t println(void);

    // EFP3 - Add printf() to make life so much easier...
    size_t printf(const char *format, ...) __attribute__ ((format (printf, 2, 3)));
    size_t printf_P(const char *format, ...) __attribute__ ((format (printf, 2, 3)));

    virtual void flush() { /* Empty implementation for backward compatibility */ }
};
// This enumeration provides the lookahead options for parseInt(), parseFloat()
// The rules set out here are used until either the first valid character is found
// or a time out occurs due to lack of input.
enum LookaheadMode{
    SKIP_ALL,       // All invalid characters are ignored.
    SKIP_NONE,      // Nothing is skipped, and the stream is not touched unless the first waiting character is valid.
    SKIP_WHITESPACE // Only tabs, spaces, line feeds & carriage returns are skipped.
};

#define NO_IGNORE_CHAR  '\x01' // a char not found in a valid ASCII numeric field

class Stream : public Print
{
  protected:
    unsigned long _timeout;      // number of milliseconds to wait for the next char before aborting timed read
    unsigned long _startMillis;  // used for timeout measurement
    int timedRead();    // private method to read stream with timeout
    int timedPeek();    // private method to peek stream with timeout
    int peekNextDigit(LookaheadMode lookahead, bool detectDecimal); // returns the next numeric digit in the stream or -1 if timeout

  public:
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;

    Stream() {_timeout=1000;}

// parsing methods

  void setTimeout(unsigned long timeout);  // sets maximum milliseconds to wait for stream data, default is 1 second
  unsigned long getTimeout(void) { return _timeout; }
  
  bool find(const char *target);   // reads data from the stream until the target string is found
  bool find(const uint8_t *target) { return find ((const char *)target); }
  // returns true if target string is found, false if timed out (see setTimeout)

  bool find(const char *target, size_t length);   // reads data from the stream until the target string of given length is found
  bool find(const uint8_t *target, size_t length) { return find ((const char *)target, length); }
  // returns true if target string is found, false if timed out

  bool find(char target) { return find (&target, 1); }

  bool findUntil(const char *target, const char *terminator);   // as find but search ends if the terminator string is found
  bool findUntil(const uint8_t *target, const char *terminator) { return findUntil((const char *)target, terminator); }

  bool findUntil(const char *target, size_t targetLen, const char *terminate, size_t termLen);   // as above but search ends if the terminate string is found
  bool findUntil(const uint8_t *target, size_t targetLen, const char *terminate, size_t termLen) {return findUntil((const char *)target, targetLen, terminate, termLen); }

  long parseInt(LookaheadMode lookahead = SKIP_ALL, char ignore = NO_IGNORE_CHAR);
  // returns the first valid (long) integer value from the current position.
  // lookahead determines how parseInt looks ahead in the stream.
  // See LookaheadMode enumeration at the top of the file.
  // Lookahead is terminated by the first character that is not a valid part of an integer.
  // Once parsing commences, 'ignore' will be skipped in the stream.

  float parseFloat(LookaheadMode lookahead = SKIP_ALL, char ignore = NO_IGNORE_CHAR);
  // float version of parseInt

  size_t readBytes( char *buffer, size_t length); // read chars from stream into buffer
  size_t readBytes( uint8_t *buffer, size_t length) { return readBytes((char *)buffer, length); }
  // terminates if length characters have been read or timeout (see setTimeout)
  // returns the number of characters placed in the buffer (0 means no valid data found)

  size_t readBytesUntil( char terminator, char *buffer, size_t length); // as readBytes with terminator character
  size_t readBytesUntil( char terminator, uint8_t *buffer, size_t length) { return readBytesUntil(terminator, (char *)buffer, length); }
  // terminates if length characters have been read, timeout, or if the terminator character  detected
  // returns the number of characters placed in the buffer (0 means no valid data found)

  // Arduino String functions to be added here
  String readString();
  String readStringUntil(char terminator);

  protected:
  long parseInt(char ignore) { return parseInt(SKIP_ALL, ignore); }
  float parseFloat(char ignore) { return parseFloat(SKIP_ALL, ignore); }
  // These overload exists for compatibility with any class that has derived
  // Stream and used parseFloat/Int with a custom ignore character. To keep
  // the public API simple, these overload remains protected.

  struct MultiTarget {
    const char *str;  // string you're searching for
    size_t len;       // length of string you're searching for
    size_t index;     // index used by the search routine.
  };

  // This allows you to search for an arbitrary number of strings.
  // Returns index of the target that is found first or -1 if timeout occurs.
  int findMulti(struct MultiTarget *targets, int tCount);
};

class HardwareSerial : public Stream
{
  public:
    virtual void begin(unsigned long) = 0;
    virtual void begin(unsigned long baudrate, uint16_t config) = 0;
    virtual void end() = 0;
    virtual int available(void) = 0;
    virtual int peek(void) = 0;
    virtual int read(void) = 0;
    virtual void flush(void) = 0;
    virtual size_t write(uint8_t) = 0;
    using Print::write; // pull in write(str) and write(buf, size) from Print
    virtual operator bool() = 0;
};

class SerialUSB : public HardwareSerial {
public:
    SerialUSB() { }
    void begin(unsigned long baud = 115200) override;
    void begin(unsigned long baud, uint16_t config) override {
        (void) config;
        begin(baud);
    };
    void end() override;

    virtual int peek() override;
    virtual int read() override;
    virtual int available() override;
    virtual int availableForWrite() override;
    virtual void flush() override;
    virtual size_t write(uint8_t c) override;
    virtual size_t write(const uint8_t *p, size_t len) override;
    using Print::write;
    operator bool() override;
    bool dtr();
    bool rts();

    void ignoreFlowControl(bool ignore = true);

    // ESP8266 compat
    void setDebugOutput(bool unused) {
        (void) unused;
    }

private:
    bool _running = false;
    bool _ignoreFlowControl = false;
};

#endif