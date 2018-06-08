#include "arduinoSerial.h"

int main() {
  Arduino my_arduino("/dev/ttyUSB0", 9600);
  float encoder;

  while(1) {
    printf("#%ld\n", my_arduino.count++);

    
    int16_t v = -(int16_t)(my_arduino.count % 255);
    printf("velocity = %d\n", v);
    my_arduino.send_to_mc(v);
    
    // 20ms wait time (=50Hz)
    usleep(1000000);
    
    encoder = my_arduino.read_from_mc();
    printf("encoder = %f\n", encoder);
  }

  my_arduino.close_serial();
}