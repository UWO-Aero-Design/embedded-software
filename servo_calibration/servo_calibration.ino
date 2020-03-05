#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"

// change these values and reupload
#define SERVOMIN 250
#define SERVOMAX 400

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x47);

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);
}

void loop() {
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(0, 0, pulselen);
  }
  delay(500);
  
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(0, 0, pulselen);
  }
  delay(500);
}
