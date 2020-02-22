/** \file CompSystem.hpp
 * @brief Fully-fledged system that will be used during competiton
 */

#pragma once

#include "System.hpp"
#include <stdio.h>


/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************** FULL SYSTEM USED FOR COMPETITION  ********************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
/*!
  @brief Implementation of a competition system
*/
class CompSystem : public System {
  
public:
    // Description of the system for printing
    static constexpr const char* DESCRIPTION = "Competition System";
    
    CompSystem() {}

    // Init method starts serial and builds test data
    bool init() override {
        Wire.begin();
        bool is_success = true;
        
        // Serial object initialization
        if(imu.init()) {
          Serial.println("IMU online.");
        }
        else {
          Serial.println("Error connecting to IMU.");
          is_success = false;
        }
        if(enviro.init()) {
          Serial.println("Environment sensor online.");
        }
        else {
          Serial.println("Error connecting to environment sensor.");
          is_success = false;
        }
        if(pitot.init()) {
          Serial.println("Pitot tube online.");
        }
        else {
          Serial.println("Error connecting to pitot tube.");
          is_success = false;
        }
        if(radio.init()) {
          Serial.println("Radio online.");
        }
        else {
          Serial.println("Error connecting to radio.");
          is_success = false;
        }
        if(servos.init()) {
          Serial.println("Servo controller online.");
        }
        else {
          Serial.println("Error connecting to servo controller.");
          is_success = false;
        }
        pinMode(20, OUTPUT);
        return is_success;
    }

    bool update() override {
      bool imu_success = imu.update();
      bool pitot_success = pitot.update();
      bool enviro_success = enviro.update();

      // collect data from sensors
      imu_data = imu.data();
      pitot_data = pitot.data();
      enviro_data = enviro.data();

      // fill print buffer with formatted text
      sprintf(print_buffer, "IMU [YPR]: %-7.2f %-7.2f %-7.2f\tPitot: %4i\tEnviro [A/T]: %-7.2f %-7.2f",
            imu_data.yaw/100.0, imu_data.pitch/100.0, imu_data.roll/100.0,
            pitot_data.differential_pressure,
            enviro_data.altitude/100.0, enviro_data.temperature/100.0);
            
      Serial.println(print_buffer);


      msg_handler.add_imu(imu_data);
      RawMessage_t response_to_gnd = msg_handler.build(aero::def::ID::Plane, aero::def::ID::Gnd, true);
      aero::def::ParsedMessage_t* incoming_msg = radio.receive(response_to_gnd);

      if(incoming_msg != NULL) {
        Serial.print("Response received from ");
        if(incoming_msg->m_from == aero::def::ID::Gnd) Serial.println("Ground Station");
        else if(incoming_msg->m_from == aero::def::ID::Plane) Serial.println("Plane");
        else Serial.println("Unknown");

        // parse commands
        aero::def::Commands_t* commands = incoming_msg->cmds();
        // check to see if any commands have been received before bitmasking to determine which are set
        if(commands != NULL) {
          run_commands(commands);
        }
      }
      else {
        Serial.println("No Message Received");
      }
      
      delay(500);
      return imu_success && pitot_success && enviro_success;
    }
    
protected:
  
private:
    // holds the nicely formatted sensor data string for printing
    char print_buffer[256];

    // Structs for data
    aero::def::IMU_t imu_data;
    aero::def::Pitot_t pitot_data;
    aero::def::GPS_t gps_data;
    aero::def::Enviro_t enviro_data;
    aero::def::Battery_t batt_data;
    aero::def::SystemConfig_t system_config_data;
    aero::def::Status_t status_state_data;
    aero::def::Servos_t servos_data;
    aero::def::AirData_t airdata_data;
    aero::def::Commands_t commands_data;
    aero::def::DropAlgo_t dropalgo_data;

    // Message handler
    aero::Message msg_handler;

    // Servo controller
    ServoController servos;

    // Sensors
    ImuMpu9250 imu;
    PhidgetPitotTube pitot { aero::teensy35::A9_PWM };
    Mpl3115a2EnviroSensor enviro;
    RFM95WServer radio{ aero::teensy35::P10_PWM, aero::teensy35::P34, aero::teensy35::P31 };

    // bitmasks for Commands_t struct of ParsedMessage_t
    const uint8_t OPEN_DOORS_MASK      = 0x01;
    const uint8_t PAYLOAD0_DROP_MASK   = 0x02;
    const uint8_t PAYLOAD1_DROP_MASK   = 0x03;
    const uint8_t PAYLOAD2_DROP_MASK   = 0x04;
    const uint8_t GLIDER0_DROP_MASK    = 0x05;
    const uint8_t GLIDER1_DROP_MASK    = 0x06;
    const uint8_t PAYLOAD0_RESET_MASK  = 0x07;
    const uint8_t PAYLOAD1_RESET_MASK  = 0x08;
    const uint8_t PAYLOAD2_RESET_MASK  = 0x09;
    const uint8_t GLIDER0_RESET_MASK   = 0x10;
    const uint8_t GLIDER1_RESET_MASK   = 0x11;
    const uint8_t CLOSE_DOORS_MASK     = 0x12;

    void run_commands(aero::def::Commands_t* commands) {
      if(commands->drop & OPEN_DOORS_MASK) {
        Serial.println("[CMD] Opening Doors");
        servos.actuate(DOOR);
      }
      if(commands->drop & PAYLOAD0_DROP_MASK) {
        Serial.println("[CMD] Dropping Payload0");
        servos.actuate(PAYLOAD0);
      }
      if(commands->drop & PAYLOAD1_DROP_MASK) {
        Serial.println("[CMD] Dropping Payload1");
        servos.actuate(PAYLOAD1);
      }
      if(commands->drop & PAYLOAD2_DROP_MASK) {
        Serial.println("[CMD] Dropping Payload2");
        servos.actuate(PAYLOAD2);
      }
      if(commands->drop & GLIDER0_DROP_MASK) {
        Serial.println("[CMD] Dropping Glider0");
        servos.actuate(GLIDER0);
      }
      if(commands->drop & GLIDER1_DROP_MASK) {
        Serial.println("[CMD] Dropping Glider1");
        servos.actuate(GLIDER1);
      }
      if(commands->drop & PAYLOAD0_RESET_MASK) {
        Serial.println("[CMD] Resetting Payload0");
        servos.reset(PAYLOAD0);
      }
      if(commands->drop & PAYLOAD1_RESET_MASK) {
        Serial.println("[CMD] Resetting Payload1");
        servos.reset(PAYLOAD1);
      }
      if(commands->drop & PAYLOAD2_RESET_MASK) {
        Serial.println("[CMD] Resetting Payload2");
        servos.reset(PAYLOAD2);
      }
      if(commands->drop & GLIDER0_RESET_MASK) {
        Serial.println("[CMD] Resetting Glider0");
        servos.reset(GLIDER0);
      }
      if(commands->drop & GLIDER1_RESET_MASK) {
        Serial.println("[CMD] Resetting Glider1");
        servos.reset(GLIDER1);
      }
      if(commands->drop & CLOSE_DOORS_MASK) {
        Serial.println("[CMD] Closing Doors");
        servos.reset(DOOR);
      }
    }
};
