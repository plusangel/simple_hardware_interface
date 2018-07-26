/**
 *
 * @file arduinoSerial.h
 *
 * \class Arduino
 * 
 * @brief This class handles the custom message communication with arduino built on
 * top of the wiringSerial library
 *
 * @author Angelos Plastropoulos
 * @date 24.07.2018
 * @todo eliminate the hardcoded values (array sizes)
 *
 */

#ifndef ARDUINOSERIAL_H
#define ARDUINOSERIAL_H

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
  /** @brief Counts the number of messages exchanged */
  unsigned long int count;
  
  /**
   * @brief This creates a new communication lonk with the Arduino board
   * @param device The linux /dev/ttyACMx port that the Arduino is connected to
   * @param baud The baud rate of the communication
   */
  Arduino(const char *device, const int baud);
  
  /**
   * @brief This method adds two integers.
   * @param vel The angular velocity (PWM) for the motor 
   */
  void send_to_mc(int16_t vel);
  
  /**
   * @brief This method reads the encoder as it is sent from the Arduino
   * @return The encoder value as float 
   */
  float read_from_mc();

  /** @brief Closes the serial communication */
  void close_serial();

private:
  /** @brief File descriptor */
  int fd;
  

  /**
   * @brief Union to have a float represented as a bytes array 
   */
  union un_float32{
    float f;
    unsigned char b[4];
  } u;
  
  /**
   * @brief Union to have a int16 represented as a bytes array 
   */
  union un_int16 {
    int16_t i;
    unsigned char b[2];
  } x;
  
  /** @brief Bytes buffer for input */
  uint8_t inputBuffer[6];

  /** @brief Bytes buffer for output */
  uint8_t outputBuffer[4];
};

#endif // ARDUINOSERIAL_H
