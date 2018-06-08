#include "arduinoSerial.h"

Arduino::Arduino(const char *device, const int baud) {
  count = 0;

  if ((fd = serialOpen (device, baud)) < 0)
  {
    fprintf (stderr, "[ArduinoSerial]: Unable to open serial device: %s\n", strerror (errno)) ;
	throw -1;
  } else {
    fprintf (stderr, "[ArduinoSerial]: Open serial device on %s at %d\n", device, baud) ;
  }
}

float Arduino::read_from_mc()
{
  float encoder = 0.0;
  // read
  //printf("*%d\n", serialDataAvail (fd));
  while(serialDataAvail (fd) >= 6) {
    int c = serialGetchar(fd);
	//printf("**%c\n", (char)c);
    if (c == 60) {
      read(fd, inputBuffer, 5);

      u.b[3] = inputBuffer[3];
      u.b[2] = inputBuffer[2];
      u.b[1] = inputBuffer[1];
      u.b[0] = inputBuffer[0];
	  encoder = u.f;
	  /*
	  for(int i = 5; i < 5; i++) {
		printf("%u ", (unsigned char)inputBuffer[i]);
	  }
	  printf("\n");
	  */
	  
      serialFlush(fd);
	  fflush (stdout) ;
    }
  }
  return encoder;
}

//-----

void Arduino::send_to_mc(int16_t input)
{
  //startMarker
  outputBuffer[0] = '<';

  //send the velocity
  x.i = input;
  outputBuffer[1] = x.b[0];
  outputBuffer[2] = x.b[1];
  
  //endMarker
  outputBuffer[3] = '>';

  write(fd, outputBuffer, 4);
}

//-----

void Arduino::close_serial ()
{
  close (fd) ;
}
