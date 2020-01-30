/** \file Radio.hpp
 * @brief All code relating to the radio
 */

#pragma once

#include "Arduino.h"
#include "src/Rfm95w/RH_RF95.h"
#include "src/aero-cpp-lib/include/Pins.hpp"
#include "src/aero-cpp-lib/include/Data.hpp"
#include "src/aero-cpp-lib/include/Message.hpp"

/*!
 * @brief Class for the RFM95W Breakout Radio by Adafruit
 * @details Communication to the radio is done via SPI
 */
class Rfm95w {
  public:
    Rfm95w() {
      rf95 = new RH_RF95(aero::teensy35::SPI0.CS, INTERRUPT_PIN);
    };
    ~Rfm95w() {
      delete rf95;
    };

    /**
     * @brief Initialize the radio
     */
    bool init() {
      pinMode(RESET_PIN, OUTPUT);
    
      // Manual radio reset
      digitalWrite(RESET_PIN, LOW);
      delay(10);
      digitalWrite(RESET_PIN, HIGH);
      delay(10);
    
      m_initialized = rf95->init();
      return m_initialized;
    };

    /**
     * @brief Send data via radio
     * 
     * @param buf A RawMessage_t containing the message to be sent
     */
    bool send(aero::def::RawMessage_t msg) {
      if(!m_initialized) {
        return false;
      }  
      rf95->send((char *)&msg, sizeof(msg));
      rf95->waitPacketSent();
      return true;
    };
    
  private: 
    RH_RF95 *rf95;
    const unsigned int RESET_PIN = aero::teensy35::P32;
    const unsigned int INTERRUPT_PIN = aero::teensy35::P32;
    
    // Track whether radio has been initialized
    bool m_initialized = false;
};
