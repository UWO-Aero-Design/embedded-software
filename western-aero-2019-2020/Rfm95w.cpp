#include "Rfm95w.h"
#define led 33 // temp

Rfm95w::Rfm95w() {
  rf95 = new RH_RF95(SPI0.CS, INTERRUPT_PIN);
}

Rfm95w::~Rfm95w() {
  delete rf95;
}


void Rfm95w::init() {
  pinMode(RESET_PIN, OUTPUT);
  pinMode(led, OUTPUT); // temp

  // Manual radio reset
  digitalWrite(RESET_PIN, LOW);
  delay(10);
  digitalWrite(RESET_PIN, HIGH);
  delay(10);

  if(!rf95->init()) {
    Serial.println("LoRa radio init failed");
  }
  else {
    Serial.println("LoRa radio init OK!");
  }
}

void Rfm95w::send(char *buf) {
  digitalWrite(led, HIGH);    
  rf95->send((char *)&buf, sizeof(buf));
  rf95->waitPacketSent();
  digitalWrite(led, LOW);
}
