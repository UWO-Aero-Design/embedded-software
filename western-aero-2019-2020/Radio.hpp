#pragma once

#include "src/Rfm95w/RH_RF95.h"
#include "src/aero-cpp-lib/include/Pins.hpp"
#include "src/aero-cpp-lib/include/Data.hpp"
#include "src/aero-cpp-lib/include/Message.hpp"

/**
 * @brief RFM95W base class
 * @details Used as a base class for RFM95WServer and RFM95WClient. Can also be used to test RFM95W initialization
 */
class RFM95W {
public:
  
  /**
   * @brief Construct a new RFM95W object
   * 
   * @param cs_pin Radio Chip Select Pin
   * @param rst_pin Radio Reset Pin
   * @param int_pin Radio Interrupt Pin
   */
  RFM95W(aero::Pin cs_pin, aero::Pin rst_pin, aero::Pin int_pin) : radio(cs_pin, int_pin) {
    this->m_rst_pin = rst_pin;
  } 

  /**
   * @brief Initialize the RFM95W radio
   * 
   * @return true if the radio successfully initialized
   * @return false if the radio failed to initialize
   */
  bool init() {
    // Set the reset pin
    pinMode(m_rst_pin, OUTPUT);
    digitalWrite(m_rst_pin, LOW);
    delay(10);
    digitalWrite(m_rst_pin, HIGH);
    delay(10);

    // Initialize the radio
    if (!radio.init()) {
      return false;
    }

    // Set radio frequency
    if (!radio.setFrequency(RADIO_FREQ)) {
      return false;
    }

    // Set radio power
    radio.setTxPower(RADIO_POWER, false);

    return true;
  }

protected:
  // Radio object
  RH_RF95 radio;

  // Handler to build and parse messages
  aero::Message message_handler;

  // Radio reset pin
  unsigned int m_rst_pin;

  // Last message received; parsed
  aero::def::ParsedMessage_t *last_recv;

  // Last message received as a buffer
  uint8_t inc_data[RH_RF95_MAX_MESSAGE_LEN] = {0};
  uint8_t inc_data_len = sizeof(inc_data);

private:
  static constexpr float RADIO_FREQ = 915.0f;
  // Max power
  static constexpr int RADIO_POWER = 23;
    
};

/**
 * @brief RFM95W object used as a client
 * @details A client will sent messages to all servers and wait for their responses
 */
class RFM95WClient : public RFM95W {
public:

  /**
   * @brief Construct a new RFM95W Client object
   * 
   * @param cs_pin Radio Chip Select Pin
   * @param rst_pin Radio Reset Pin
   * @param int_pin Radio Interrupt Pin
   */
  RFM95WClient(aero::Pin cs_pin, aero::Pin rst_pin, aero::Pin int_pin): RFM95W(cs_pin, rst_pin, int_pin) {} 
  
  /**
   * @brief Set the timeout
   * 
   * @param int new timeout value
   */
  void setTimeout(const unsigned int& timeout) {
    m_timeout = timeout;
  }

  /**
   * @brief Send a message from the client and return the response from the appropriate server
   * 
   * @param message Message to send from the client to all active devices
   * @return aero::def::ParsedMessage_t* Response from server that recevied the message
   */
  aero::def::ParsedMessage_t* send(aero::def::RawMessage_t message) {
    // Send packet
    bool valid = radio.send((char *)&message, sizeof(message));
    radio.waitPacketSent();

    if(!valid) {
      return NULL;
    }

    if (radio.waitAvailableTimeout(m_timeout)) { 
      if (radio.recv(inc_data, &inc_data_len)) {
        // Parse response and return it
        return message_handler.parse(inc_data);
      } else {
        return NULL;
      }
    } else {
      return NULL;
    }
  }

  aero::def::ParsedMessage_t* send(char* buf, int len) {
    // Send packet
    bool valid = radio.send(buf, len);
    radio.waitPacketSent();

    if(!valid) {
      return NULL;
    }

    if (radio.waitAvailableTimeout(m_timeout)) { 
      if (radio.recv(inc_data, &inc_data_len)) {
        // Parse response and return it
        return message_handler.parse(inc_data);
      } else {
        return NULL;
      }
    } else {
      return NULL;
    }
  }

  int16_t rssi() {
    return radio.lastRssi();
  }

private:
  // Timeout that defines how long the client will wait for a valid response from the servers
  static constexpr unsigned int DEFAULT_TIMEOUT = 3000;
  unsigned int m_timeout = DEFAULT_TIMEOUT;

};

/**
 * @brief RFM95W object used as a server
 * @details A server will wait for messages from a client and respond accordingly
 */
class RFM95WServer : public RFM95W {
public:

  /**
   * @brief Construct a new RFM95W Server object
   * 
   * @param cs_pin Radio Chip Select Pin
   * @param rst_pin Radio Reset Pin
   * @param int_pin Radio Interrupt Pin
   */
  RFM95WServer(aero::Pin cs_pin, aero::Pin rst_pin, aero::Pin int_pin): RFM95W(cs_pin, rst_pin, int_pin) {} 
  
  /**
   * @brief Recieve new message from radio if one is available. If message is available, send the appropriate response
   * @details NOTE: problem is that the response is deteremined before the message. Could fix this but seems like a later problem
   * @param response Response to send after receiving new message
   * @return aero::def::ParsedMessage_t* The message received from client
   */
//  aero::def::ParsedMessage_t* receive(aero::def::RawMessage_t response) {
//    if (radio.available()) {
//      if (radio.recv(inc_data, &inc_data_len)) {
//        
//        // Send a reply
//        bool valid = radio.send((char *)&response, sizeof(response));
//        radio.waitPacketSent();
//
//        if(!valid) {
//          return NULL;
//        } else {
//          // Parse response and return it
//          for(int i = 0; i < sizeof(inc_data)/sizeof(uint8_t); i++) {
//            if(i%32 == 0) Serial.println();
//            if(inc_data[i] == 0) Serial.print("0 ");
//            else Serial.print(inc_data[i], HEX);
//            Serial.print(" ");
//          }
//          Serial.println();
//          return message_handler.parse(inc_data);
//        }
//      } else {
//        return NULL;
//      }
//    } else {
//      return NULL;
//    }
//  }

bool receive(aero::def::ParsedMessage_t** msg) {
  if (radio.available()) {
      if (radio.recv(inc_data, &inc_data_len)) {
              // Parse response and return it
              *msg = message_handler.parse(inc_data);
              Serial.print("Message from: "); Serial.print(static_cast<int>((*msg)->m_from));
              Serial.print(" to: "); Serial.println(static_cast<int>((*msg)->m_to));
//              msg = last_recv;
              return true;
      } else {
          return false;
      }
  } else {
      return false;
  }
}

  bool respond(aero::def::RawMessage_t response) {
  // Send a reply
  bool valid = radio.send((char *)&response, sizeof(response));
  radio.waitPacketSent();
  return valid;
}

  private:
};
