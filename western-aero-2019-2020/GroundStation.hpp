#pragma once

#include "System.hpp"

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/************************ SYSTEM FOR GROUND STATION ************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
class GroundStation : public System {
public:

  // Description string
  static constexpr const char* DESCRIPTION = "Ground Station Demo";

  bool init() override {
    // Start serial port for reading from groundstation
    Serial.begin(115200);

    bool success = radio.init();

    if(!success) {
      return false;
    }
    else {
      return true;
    }
  }

  bool update() override {
    // Check if there is a new serial message that has arrived
    bool new_serial_msg = serial::check_for_msg(Serial);

    // Relay the message off
    if(new_serial_msg == true) {
        // If a new serial message is found, send it over the radio

        // Grab valid message contents and length
        int len = serial::msg_contents(buffer);

        // Parse the serial message to check who the message is for
        ParsedMessage_t* result = msg_handler.parse(buffer);
        
        // If the message is to the plane, track the time it was sent
        if(result->m_to == ID::Plane) {
            m_last_plane_msg_ms = millis();
        }

        // Send the buffer over radio to devices
        ParsedMessage_t* server_response = radio.send(buffer, len);

        // Relay response
        if(server_response != NULL) {
            send_to_pc(server_response);
        }

    } else {
        // Make sure to send a message to the plane every second if one does not come in from PC
        unsigned long curr = millis();

        if(curr - m_last_plane_msg_ms >= 1000) {
            m_last_plane_msg_ms = curr;

            // Send empty message to plane, to request data
            RawMessage_t msg_to_send = msg_handler.build(aero::def::ID::Gnd, aero::def::ID::Plane, true);
            ParsedMessage_t* server_response = radio.send(msg_to_send);

            // If plane responds, relay response to groundstation
            if(server_response != NULL) {
                send_to_pc(server_response);
            }
        }

        delay(100);
    }

    return true;

  }
private:
  aero::Message msg_handler;

  void send_to_pc(ParsedMessage_t* msg) {
     // Data. to, from, last rssi, rest of the data
    if(msg->imu() != NULL) 
        msg_handler.add_imu(*(msg->imu()));

    if(msg->pitot() != NULL)
        msg_handler.add_pitot(*(msg->pitot()));

    if(msg->gps() != NULL)
        msg_handler.add_gps(*(msg->gps()));

    if(msg->enviro() != NULL)
        msg_handler.add_enviro(*(msg->enviro()));

    if(msg->battery() != NULL)
        msg_handler.add_battery(*(msg->battery()));

    if(msg->config() != NULL)
        msg_handler.add_config(*(msg->config()));

    if(msg->status() != NULL) {
        msg->status()->rssi = (int16_t) radio.rssi();
        msg_handler.add_status(*(msg->status()));
    } else {
        def::Status_t status;
        status.rssi = (int16_t) radio.rssi();
    }

    if(msg->servos() != NULL)
        msg_handler.add_actuators(*(msg->servos()));

    if(msg->air_data() != NULL)
        msg_handler.add_airdata(*(msg->air_data()));

    if(msg->cmds() != NULL)
        msg_handler.add_cmds(*(msg->cmds()));

    if(msg->drop_algo() != NULL)
        msg_handler.add_drop(*(msg->drop_algo()));
    
    // Cast buffer to check for message length
    aero::def::RawMessage_t tmp_msg = msg_handler.build(msg->m_from, msg->m_to, true);
    char* buf = (char*) &tmp_msg;

    // Send message. Make sure to skip the part of the buffer that is empty
    for(int i = 0; i < 209; ++i) {
        // Skip empty parts
        if(i == tmp_msg.length+6) {
            i = 200+6;
        }

        Serial.print((char)buf[i], HEX);
        Serial.print(' ');
    }

    Serial.print("\n");
  }

  int cs_pin =  aero::teensy35::P10_PWM;
  int rst_pin = aero::teensy35::P34;
  int int_pin = aero::teensy35::P31;

  RFM95WClient radio{ cs_pin, rst_pin, int_pin }; 

  char buffer[256];

  unsigned long m_last_plane_msg_ms = 0;
};
