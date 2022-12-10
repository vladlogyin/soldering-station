#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <algorithm>
#include <cstddef>
/**
 * Template class that implements a ring buffer with a fixed size
 *
 * @param buffersize Size of the buffer (actual size is 1 less)
 */
template <size_t buffersize>
class basic_ringbuffer
{
  /**
   * Actual internal buffer
   */
  char buffer[buffersize];
  size_t readpointer = 0;
  size_t writepointer = 0;
  /**
   * Busy flag used as a mutex
   */
  bool busy = false;
public:
  basic_ringbuffer()
  {

  }
  basic_ringbuffer(basic_ringbuffer&& o)
  {
    std::move(o.buffer, o.buffer + buffersize, buffer);
    readpointer = o.readpointer;
    writepointer = o.writepointer;
  }
  /**
   * Get number of characters
   *
   * Returns the number of characters currently in the buffer
   *
   * @returns number of bytes
   */
  size_t getavailable()
  {
    if(readpointer > writepointer)
    {
      return writepointer + buffersize - readpointer;
    }
    else
    {
      return writepointer - readpointer;
    }
  }

  /**
   * Get string from the buffer
   *
   * Gets n characters from the buffer into s
   *
   * @param s string
   * @param n number of characters
   * @returns number of characters read
   */
  size_t sgetn(char* s, size_t n)
  {
    return xsgetn(s, n);
  }

  virtual size_t xsgetn(char* s, size_t n)
  {
    busy = true;
    n = std::min(n, getavailable());
    size_t endpos = readpointer + n;

    if(endpos < buffersize)
    {
      std::move(buffer + readpointer, buffer + endpos, s);
      readpointer = endpos;
      busy = false;
      return n;
    }
    else
    {
      std::move(buffer + readpointer, buffer + buffersize, s);
      endpos = endpos - buffersize;
      std::move(buffer, buffer + endpos + 1, s + buffersize - 1 - readpointer);
      readpointer = endpos;
      busy = false;
      return n;
    }
  }

  /**
   * Append string to the buffer
   *
   * Appends n characters from s into the buffer
   *
   * @param s string
   * @param n number of characters
   * @returns number of characters written
   */
  size_t sputn(const char* s, size_t n)
  {
    return xsputn(s, n);
  }

  virtual size_t xsputn(const char* s, size_t n)
  {
    busy = true;
    n = std::min(n, buffersize - getavailable() - 1);
    size_t endpos = writepointer + n;

    if(endpos < buffersize)
    {
      std::move(s, s + n, buffer + writepointer);
      writepointer = endpos;

      busy = false;
      return n;
    }
    else
    {
      std::move(s, s + buffersize - 1 - writepointer, buffer + writepointer);
      std::move(s + buffersize - 1 - writepointer, s + n, buffer);
      writepointer = endpos - buffersize;
      busy = false;
      return n;
    }
  }

  /**
   * Overflow character
   *
   * This function is called when the buffer is already full and should be overriden in parent classes
   *
   * @returns 0 if successful, -1 if unimplemented
   */
  virtual int overflow(int c)
  {
    return -1;
  }

  /**
   * Flush buffer contents
   *
   * This virtual function should be overriden in parent classes
   *
   * @returns bytes flushed;
   */
  virtual int flush()
  {
    return -1;
  }

private:

};

typedef basic_ringbuffer<64> ringbuffer;

#endif
