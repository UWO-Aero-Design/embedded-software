#include "src/aero/aero-cpp-lib/include/Message.hpp"


#define led 13

aero::Message msg;

void setup() {
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  Serial.println("Hello World!"); 
}
void loop() {
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
  delay(1000);
}
