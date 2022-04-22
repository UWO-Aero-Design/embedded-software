#include "src/Rfm95w/RH_RF95.h"

#define CONNECT_FLAG 0xAA
#define PACKET_FLAG 0xBB
#define SERIAL_DEBOUNCE 100

long last_update = 0;

const int LED = 33;
const int RESET_PIN = 8;

RH_RF95 radio = RH_RF95 (10, 9);
float RADIO_FREQ = 905.0f;

void setup() {
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  // manual reset hack
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  delay(10);
  digitalWrite(RESET_PIN, HIGH);
  delay(10);

  if(!radio.init()) {
    digitalWrite(LED, HIGH);
    Serial.println("Radio init failed");
    while(1);
  }

  if (!radio.setFrequency(RADIO_FREQ)) {
    digitalWrite(LED, HIGH);
    Serial.println("Radio frequency set failed");
    while(1);
  }
    
  digitalWrite(LED, HIGH);
  delay(300);
  digitalWrite(LED, LOW);
  Serial.println("Waiting for serial activity...");
}

void loop() {
  if(Serial.available() > 0 || millis() - last_update > SERIAL_DEBOUNCE) {
    digitalWrite(LED, HIGH);
    int len = Serial.available();
    uint8_t data[len];
    for(int i = 0; i < len; i++) {
      data[i] = Serial.read();
    }

    if(data[0] == CONNECT_FLAG) {
      Serial.write(CONNECT_FLAG);
      digitalWrite(LED, HIGH);
      delay(500);
      digitalWrite(LED, LOW);
    }
    else {
      if(len == 1) return;
      radio.send(&data[1], len-1);
    }
    digitalWrite(LED, LOW);
    last_update = millis();
  }
  
  if (radio.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (radio.recv(buf, &len)) {
      digitalWrite(LED, HIGH);

      // write to serial
      for(int i = 0; i < len; i++) {
        Serial.write(buf[i]);
      }

      // send rssi as well - don't want to decode packet just for this
//      int16_t rssi = radio.lastRssi();
//      Serial.write(highByte(rssi));
//      Serial.write(lowByte(rssi));
      
      digitalWrite(LED, LOW);
    }
    else {
      Serial.println("recv failed");
    }
    
  }
}
