#include "TestMessages.h"

#define led 13

using namespace aero;
using namespace aero::def;

Message msg_handler;


IMU_t imu;
Pitot_t pitot = random_pitot();

void setup() {
  imu = random_imu();
  //pitot = 
  
  // Init serial port
  Serial.begin(115200);

  // Need to seed random number generator from setup()
  randomSeed(analogRead(0));

  pinMode(led, OUTPUT);
  Serial.println("Hello World!");
  
}
void loop() {
  // Generate random data
  
  pitot.differential_pressure += 100;
  // Create message buffer
  msg_handler.add_imu(imu);
  
  msg_handler.add_pitot(pitot);
  
  RawMessage_t raw_msg = msg_handler.build(ID::G1, ID::G2);
  char *buf = (char *) &raw_msg;

  // Send message. Make sure to skip the part of the buffer that is empty
  for(int i = 0; i < sizeof(raw_msg); ++i) {
    // Skip empty parts of buffer
    if(i == raw_msg.length+6) {
      i = 256+6;
    }

    Serial.print((char)buf[i], HEX);
    Serial.print(' ');
  }
  
  Serial.print("\n");

  // Message every second
  delay(1000);

  ParsedMessage_t parsed = msg_handler.parse((uint8_t *)buf);
  IMU_t *imuu = ( reinterpret_cast<IMU_t*>( parsed.segments[ static_cast<int>(Signature::IMU) ] ) );
  //Serial.print("Hello: ");
  //Serial.print( imuu->ax );
  //Serial.print( ' ' );
  //Serial.println( imuu->gy );
}
