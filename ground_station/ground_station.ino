#include "src/Rfm95w/RH_RF95.h"

#define CONNECT_FLAG 0xAA
#define PACKET_FLAG 0xBB
#define SERIAL_DEBOUNCE 100
#define RADIO_LINK_TIMEOUT 1000

bool radio_link_connection = false;
long last_radio_ping = 0;
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
    int len = Serial.available();
    uint8_t data[len];
    for(int i = 0; i < len; i++) {
      data[i] = Serial.read();
    }

    if(data[0] == CONNECT_FLAG) {
      Serial.write(CONNECT_FLAG);
    }
    else {
      if(len == 1) return;
      radio.send(&data[1], len-1);
    }
    last_update = millis();
  }
  
  if (radio.available()) {
    last_radio_ping = millis();
    if(radio_link_connection == false) {
      radio_link_connection = true;
      digitalWrite(LED, HIGH);
    }
    
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (radio.recv(buf, &len)) {

      // write to serial
      for(int i = 0; i < len; i++) {
        Serial.write(buf[i]);
      }
      
    }
    else {
      Serial.println("recv failed");
    }
    
  }

  if(radio_link_connection == true && millis() - last_radio_ping >= RADIO_LINK_TIMEOUT) {
    digitalWrite(LED, LOW);
    radio_link_connection = false;
  }
}
