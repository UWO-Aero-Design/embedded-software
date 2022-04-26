#include "src/Rfm95w/RH_RF95.h"
#include "src/Message/pb_encode.h"
#include "src/Message/pb_decode.h"
#include "src/Message/telemetry.pb.h"

#define CONNECT_FLAG 0xAA
#define PACKET_FLAG 0xBB
#define SERIAL_DEBOUNCE 100
#define RADIO_LINK_TIMEOUT 1000
#define BUFFER_SIZE 128

bool radio_link_connection = false;
long last_radio_ping = 0;
long last_update = 0;

const int LED = 33;
const int RESET_PIN = 8;

RH_RF95 radio = RH_RF95 (10, 9);
float RADIO_FREQ = 905.0f;

Telemetry telemetry;

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
    
    uint8_t receive_buffer[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t receive_buffer_length = sizeof(receive_buffer);

    if (radio.recv(receive_buffer, &receive_buffer_length)) {
        
      telemetry = Telemetry_init_zero;
      pb_istream_t decode_stream = pb_istream_from_buffer(receive_buffer, receive_buffer_length);
      bool status = pb_decode(&decode_stream, Telemetry_fields, &telemetry);

      if(status) {
        telemetry.gnd_radio.rssi = radio.lastRssi();
        telemetry.gnd_radio.frequency_error = radio.frequencyError();
        telemetry.gnd_radio.snr = radio.lastSNR();
        telemetry.has_gnd_radio = true;

        uint8_t encode_buffer[BUFFER_SIZE];
        uint8_t encode_buffer_length = sizeof(encode_buffer);
        for(int i = 0; i < encode_buffer_length; i++) encode_buffer[i] = 0;
        
        pb_ostream_t encode_stream = pb_ostream_from_buffer(encode_buffer, encode_buffer_length);
        bool status = pb_encode(&encode_stream, Telemetry_fields, &telemetry);
        
        if(status) {
          for(int i = 0; i < encode_stream.bytes_written; i++) {
            Serial.write(encode_buffer[i]);
          }
        }
        else {
          digitalWrite(LED, HIGH);
          delay(3000);
          digitalWrite(LED, LOW);
          delay(3000);
        }
      }
      else {
        digitalWrite(LED, HIGH);
        delay(1000);
        digitalWrite(LED, LOW);
        delay(1000);
      }

      // write to serial
//      for(int i = 0; i < receive_buffer_length; i++) {
//        Serial.write(receive_buffer[i]);
//      }
      
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
