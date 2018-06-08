#include "config.h"

// struct of the outgoing package inlcuding encoders
typedef struct {
  char startMark;
  float encoder;
  char endMark;
} __attribute__((__packed__))data_packet_t; 

data_packet_t hub;

const int baud_rate = SERIAL_BAUD_RATE;
const byte numBytes = NUM_OF_INCOMING_BYTES;
const byte numPayloadBytes = numBytes - 2;
byte receivedBytes[numBytes];

union velocity {
  int16_t value;
  byte bytes[2];
} requested_velocity;

boolean newData = false;

int count = 0;
unsigned long my_time;

void setup()                    
{
  // begin serial in programming USB serial for debugging
  Serial.begin(SERIAL_BAUD_RATE);
  // begin serial in native USB for connection with rPi3
  SerialUSB.begin(SERIAL_BAUD_RATE);           
  
  // initialisation
  hub.startMark = '<';
  hub.encoder = 0;
  hub.endMark = '>';

  /* turn on the timer clock in the power management controller */
  pmc_set_writeprotect(false);     // disable write protection for pmc registers
  pmc_enable_periph_clk(ID_TC7);   // enable peripheral clock TC7

  /* we want wavesel 01 with RC */
  TC_Configure(/* clock */TC2,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4); 
  TC_SetRC(TC2, 1, 65625); //(=> 20ms period/1000 = 0.02s which we multiply with 656250 = 13250)
  TC_Start(TC2, 1);

  // enable timer interrupts on the timer
  TC2->TC_CHANNEL[1].TC_IER=TC_IER_CPCS;   // IER = interrupt enable register
  TC2->TC_CHANNEL[1].TC_IDR=~TC_IER_CPCS;  // IDR = interrupt disable register

  /* Enable the interrupt in the nested vector interrupt controller */
  /* TC4_IRQn where 4 is the timer number * timer channels (3) + the channel number (=(1*3)+1) for timer1 channel1 */
  NVIC_EnableIRQ(TC7_IRQn);

}

void TC7_Handler()
{
  // We need to get the status to clear it and allow the interrupt to fire again
  TC_GetStatus(TC2, 1);
    
  // send to on-board computer the encoder value
  data_send();

  // ---- Receive the velocity command (ROS hardware interface) 
  data_receive();

  // ---- Log only the input values ----
  newDataAvailable();

  my_time = millis();
  Serial.println(my_time);
}

void loop()                       
{   
  
}

//-----------------------------------------------------------
//

void data_send()
{
  hub.startMark = '<';
  hub.encoder = (float)(count % 255); // Rolling value for testing only
  hub.endMark = '>';

  logOutputData(hub);
 
  unsigned long uBufSize = sizeof(data_packet_t);
  char pBuffer[uBufSize];
  
  memcpy(pBuffer, &hub, uBufSize);
  for(int i = 0; i<uBufSize;i ++) {
   SerialUSB.write(pBuffer[i]);
   SerialUSB.flush();
  }
  

  count++;
}

//-----------------------------------------------------------
//

void logOutputData(data_packet_t t) 
{
  Serial.print("Outgoing data: ");
  Serial.println(t.encoder);
}


//-----------------------------------------------------------
//

void data_receive() 
{
  static boolean recvInProgress = false;
  static byte ndx = 0;
  byte startMarker = 0x3C;
  byte endMarker = 0x3E;
  byte rc;

  //SerialUSB.println("Debug: Intro");

  while (SerialUSB.available() > 0 && newData == false) {
    rc = SerialUSB.read();

    //SerialUSB.println("Debug: Receive bytes");
    
    if (recvInProgress == true) {

      //SerialUSB.println("Debug: Receive in progress");
      
      if (rc != endMarker) {

        //SerialUSB.println("Debug: Fill-in the array");
        
        receivedBytes[ndx] = rc;
        ndx++;
        if (ndx >= numBytes) {
          ndx = numBytes - 1;
        } 
      } else {
        //SerialUSB.println("Debug: Filled array");
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    } 
  }
}

//-----------------------------------------------------------
//

void newDataAvailable() {
  if (newData == true) {
    
    //SerialUSB.println("Debug: Valid data received");
    //SerialUSB.println("Debug: Received ");

    requested_velocity.bytes[0] = receivedBytes[0];
    requested_velocity.bytes[1] = receivedBytes[1];
    
    Serial.print("Requested velocity = ");
    Serial.println(requested_velocity.value);    
    
    newData = false;
  }
}

