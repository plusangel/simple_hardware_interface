#include "config.h"

// struct of the outgoing package inlcuding encoders
typedef struct {
  char startMark;
  float encoder;
  char endMark;
} __attribute__((__packed__))data_packet_t; 

data_packet_t hub;

//const int baud_rate = SERIAL_BAUD_RATE;
const byte numBytes = NUM_OF_INCOMING_BYTES;
const byte numPayloadBytes = numBytes - 2;
byte receivedBytes[numBytes];

union velocity {
  int16_t value;
  byte bytes[2];
} requested_velocity;

boolean newData = false;

int timer1_counter;
int count = 0;
bool sw = true;

void setup()                    
{
  // begin serial
  Serial2.begin(9600);          
  Serial.begin(9600); 
  
  // initialisation
  hub.startMark = '<';
  hub.encoder = 0;
  hub.endMark = '>';

  cli();//stop interrupts

  //set timer1 interrupt at 0.5Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts

}

ISR(TIMER1_COMPA_vect)
{
  TCNT1 = timer1_counter;   // preload timer

  //if(sw) {
  // send to on-board computer the encoder value
  data_send();
  //} else {
  // ---- Receive the velocity command (ROS hardware interface) 
  data_receive();

  // ---- Log only the input values ----
  newDataAvailable();
  //}

  //sw = !sw;
}

void loop()                       
{   
  
}

//-----------------------------------------------------------
//

void data_send()
{
  hub.startMark = '<';
  hub.encoder = (float)(count % 255);
  hub.endMark = '>';

  logOutputData(hub);
 
  unsigned long uBufSize = sizeof(data_packet_t);
  char pBuffer[uBufSize];
  
  memcpy(pBuffer, &hub, uBufSize);
  for(int i = 0; i<uBufSize;i ++) {
   Serial2.print(pBuffer[i]);
   Serial2.flush();
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

  //Serial1USB.println("Debug: Intro");

  while (Serial2.available() > 0 && newData == false) {
    rc = Serial2.read();

    //Serial1USB.println("Debug: Receive bytes");
    
    if (recvInProgress == true) {

      //Serial1USB.println("Debug: Receive in progress");
      
      if (rc != endMarker) {

        //Serial1USB.println("Debug: Fill-in the array");
        
        receivedBytes[ndx] = rc;
        ndx++;
        if (ndx >= numBytes) {
          ndx = numBytes - 1;
        } 
      } else {
        //Serial1USB.println("Debug: Filled array");
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
    
    //Serial1USB.println("Debug: Valid data received");
    //Serial1USB.println("Debug: Received ");

    requested_velocity.bytes[0] = receivedBytes[0];
    requested_velocity.bytes[1] = receivedBytes[1];
    
    Serial.print("Requested velocity = ");
    Serial.println(requested_velocity.value);    
    
    newData = false;
  }
}

