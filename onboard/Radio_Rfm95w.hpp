#pragma once

#include "src/Rfm95w/RH_RF95.h"
#include "Pins.hpp"

/**
   @brief Class for a radio using the RFM95W radio
   @details It is a LoRa radio capable of sending and receiving data over RF
   @note all messages sent to/from the radio shall be prepended with a single byte
         representing the length of the message (eg. { 5, _, _, _, _, _ })
*/
class Radio_Rfm95w : public aero::sensor::Radio {
  public:

    static constexpr const size_t RECEIVE_BUFFER_SIZE = RH_RF95_MAX_MESSAGE_LEN;

    /**
       @brief Construct a new RFM95W object
    */
    Radio_Rfm95w() {
      radio = new RH_RF95((uint8_t)Pins::RADIO_CS, (uint8_t)Pins::RADIO_INT);
      m_data.rssi = 0;
      m_data.frequencyError = 0;
      m_data.snr = 0;
      bytesReceived = 0;
    }

    /**
       @brief Initialize the RFM95W radio

       @return true if the radio successfully initialized
       @return false if the radio failed to initialize
    */
    bool init() override {
      // Set the reset pin
      pinMode(Pins::RADIO_RST, OUTPUT);
      digitalWrite(Pins::RADIO_RST, LOW);
      delay(10);
      digitalWrite(Pins::RADIO_RST, HIGH);
      delay(10);

      // Initialize the radio
      if (!radio->init()) {
        return false;
      }

      // Set radio frequency
      if (!radio->setFrequency(RADIO_FREQ)) {
        return false;
      }

      // Set radio power
         radio->setTxPower(RADIO_POWER, false);

      return true;
    }

    bool update() override {
      m_data.rssi = radio->lastRssi();
      m_data.frequencyError = radio->frequencyError();
      m_data.snr = radio->lastSNR();
      return true;
    }

    bool ready() override {
      return radio->available();
    }

    bool receive(uint8_t* buf, uint8_t* len) override {
      uint8_t bufLen = RH_RF95_MAX_MESSAGE_LEN-bytesReceived;
      if(!radio->recv(&recvBuffer[bytesReceived], &bufLen)) {
        // recv failed
        return false;
      }
      else if(bufLen == 0) {
        // empty message received
        return false;
      }

      if(bytesReceived == 0) {
        // this is the first recv() called for this message
        msgLength = recvBuffer[0];

        if(msgLength == 0 || msgLength > RH_RF95_MAX_MESSAGE_LEN) {
          return false;
        }
      }

      // increment bytes received by the number of bytes we just received
      bytesReceived += bufLen;

      // if we received the right number of bytes
      if(bytesReceived-1 >= msgLength) {
        // copy the message into the caller's buffer and return success
        for(uint8_t i = 0; i < msgLength; i++) {
          buf[i+1] = recvBuffer[i];
        }
        bytesReceived = 0;
        return true;
      }

      // partial message - need main script to call receive() again
      return false;
    }

    bool send(uint8_t* buf, uint8_t len) override {
      // verify we have enough room in our local buffer
      if(len > RH_RF95_MAX_MESSAGE_LEN-1) {
        Serial.println("Buffer overflow");
        return false;
      }

      // put the length in the first byte
      sendBuffer[0] = len;

      // copy the rest of the message into the send buffer
      for(uint8_t i = 0; i < len; i++) {
        sendBuffer[i+1] = buf[i];
      }

      // send the length and message
      return radio->send(sendBuffer, len+1);
    }

  protected:
    // Radio object
    RH_RF95* radio;

  private:
    static constexpr float RADIO_FREQ = 433 ;
    static constexpr int RADIO_POWER = 23; // Max power

    uint8_t sendBuffer[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t recvBuffer[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t msgLength = 0;
    uint8_t bytesReceived = 0;

};
