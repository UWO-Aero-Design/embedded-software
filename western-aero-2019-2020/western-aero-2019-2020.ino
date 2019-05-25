#include "libraries/MPU9250/MPU9250.h"

MPU9250 imu;

#define led 13

void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  Serial.println("Hello World!"); 
  imu.setup();
}
void loop() {
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(1000);
}
