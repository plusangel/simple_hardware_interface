#include <errno.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include "wiringSerial.h"

class Arduino {

public:
  unsigned long int count;

  Arduino(const char *, const int);
  void send_to_mc(int16_t);
  float read_from_mc();
  void close_serial();

private:
  int fd;
  
  union {
    float f;
    unsigned char b[4];
  } u;
  
  union {
    int16_t i;
    unsigned char b[2];
  } x;
  
  uint8_t inputBuffer[6];
  uint8_t outputBuffer[4];
};
