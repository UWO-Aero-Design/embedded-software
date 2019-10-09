#include "TestMessages.h"

#define led 13

using namespace aero;
using namespace aero::def;

Message msg_handler;



void setup() {
  // Init serial port
  Serial.begin(115200);

  // Need to seed random number generator from setup()
  randomSeed(analogRead(0));

  pinMode(led, OUTPUT);
  Serial.println("Hello World!");
  
}
void loop() {
  IMU_t imu = random_imu();
  msg_handler.add_imu(imu);
  RawMessage_t raw_msg = msg_handler.build(ID::G1, ID::G2);

  char *buf = (char *) &raw_msg;

  for(int i = 0; i < sizeof(raw_msg); ++i)
  {
    Serial.print((char)buf[i], HEX);
    Serial.print(' ');
  }
  
  Serial.print("\n");

  delay(1000);
  
}
