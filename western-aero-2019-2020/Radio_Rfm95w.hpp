#pragma once

#include "src/Rfm95w/RH_RF95.h"
#include "Pins.hpp"

/**
 * @brief Class for a radio using the RFM95W radio
 * @details It is a LoRa radio capable of sending and receiving data over RF
 */
class Radio_Rfm95w {
public:
  
  /**
   * @brief Construct a new RFM95W object
   */
  Radio_Rfm95w() {
    radio = new RH_RF95((uint8_t)Pins::RADIO_CS, (uint8_t)Pins::RADIO_INT);
  } 

  /**
   * @brief Initialize the RFM95W radio
   * 
   * @return true if the radio successfully initialized
   * @return false if the radio failed to initialize
   */
  bool init() {
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
//    radio->setTxPower(RADIO_POWER, false);

    return true;
  }

bool send(uint8_t* buf, uint8_t len) {
  return radio->send(buf, len);
}

protected:
  // Radio object
  RH_RF95* radio;

private:
  static constexpr float RADIO_FREQ = 905.0f;
  static constexpr int RADIO_POWER = 23; // Max power
    
};
