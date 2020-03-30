#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"

// change these values and reupload
#define SERVOMIN 190 // 470, 150
#define SERVOMAX 191
#define SERVO_NUM 3

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
    pwm.setPWM(SERVO_NUM, 0, pulselen);
    //delay(1);
  }
  delay(1000);
  
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(SERVO_NUM, 0, pulselen);
    //delay(1);
  }
  delay(1000);
}
