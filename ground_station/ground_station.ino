#include "src/Rfm95w/RH_RF95.h"
#include "src/Message/pb_encode.h"
#include "src/Message/pb_decode.h"
#include "src/Message/telemetry.pb.h"

const int LED = 22;
const int RESET_PIN = 9;

RH_RF95 radio = RH_RF95 (8, 2);

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(300);
  digitalWrite(LED, LOW);
  
  Serial.begin(115200);
  
  // manual reset hack
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(10);
  digitalWrite(RESET_PIN, HIGH);
  delay(10);

  if(!radio.init()) {
    digitalWrite(LED, HIGH);
    error();
  }

  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
}

void loop() {  
  if (radio.available()) {
    uint8_t radioReceiveBuffer[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t receiveBufferLength = RH_RF95_MAX_MESSAGE_LEN;

    if(radio.recv(radioReceiveBuffer, &receiveBufferLength)) {
      if(receiveBufferLength != 0) {
        Serial.write(radioReceiveBuffer, receiveBufferLength);
      }
    }
    else {
      error();
    }
    
  }

  if(Serial.available() >= 1) {
    uint8_t length = Serial.read();
    if(length <= RH_RF95_MAX_MESSAGE_LEN) {
      uint8_t buf[length+1];
      buf[0] = length;
      for(uint8_t i = 0; i < length; i++) {
        buf[i+1] = Serial.read();
      }
      radio.send(buf, length+1);
    }
  }
}

void error() {
  while(1) {
      digitalWrite(LED, HIGH);
      delay(1000);
      digitalWrite(LED, LOW);
      delay(1000);
    }
}
