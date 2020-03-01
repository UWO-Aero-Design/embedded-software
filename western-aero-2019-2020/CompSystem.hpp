/** \file CompSystem.hpp
   @brief Fully-fledged system that will be used during competiton
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

    CompSystem() {
      incoming_msg = new ParsedMessage_t();
    }

    ~CompSystem() {
      delete incoming_msg;
    }

    // Init method starts serial and builds test data
    bool init() override {
      Wire.begin();
      bool is_success = true;

      // Serial object initialization
      if (imu.init()) {
        Serial.println("IMU online.");
      }
      else {
        Serial.println("Error connecting to IMU.");
        is_success = false;
      }
      if (enviro.init()) {
        Serial.println("Environment sensor online.");
      }
      else {
        Serial.println("Error connecting to environment sensor.");
        is_success = false;
      }
      if (pitot.init()) {
        Serial.println("Pitot tube online.");
      }
      else {
        Serial.println("Error connecting to pitot tube.");
        is_success = false;
      }
      if (radio.init()) {
        Serial.println("Radio online.");
      }
      else {
        Serial.println("Error connecting to radio.");
        is_success = false;
      }
      if (servos.init()) {
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
              imu_data.yaw / 100.0, imu_data.pitch / 100.0, imu_data.roll / 100.0,
              pitot_data.differential_pressure,
              enviro_data.altitude / 100.0, enviro_data.temperature / 100.0);

      // Serial.println(print_buffer);


      msg_handler.add_imu(imu_data);
      msg_handler.add_pitot(pitot_data);
      msg_handler.add_enviro(enviro_data);

      RawMessage_t response_to_gnd = msg_handler.build(aero::def::ID::Plane, aero::def::ID::Gnd, true);
      digitalWrite(22, LOW);


      aero::def::ParsedMessage_t* radio_recv = radio.receive();

      // Receive the incoming message
      if ( radio_recv != NULL ) {

        if (radio_recv->m_to == aero::def::ID::Plane) {
          digitalWrite(22, HIGH);

          Serial.println("Message received from ground station");

          if (radio_recv->cmds() != NULL) {

           Serial.print("Drop: ");
           Serial.println(radio_recv->cmds()->drop, BIN);
           Serial.print("Servos: ");
           Serial.println(radio_recv->cmds()->servos, BIN);
           Serial.print("Pitch: ");
           Serial.println(radio_recv->cmds()->pitch, BIN);

           //run_servos(incoming_msg->cmds());

         }

         radio.respond(response_to_gnd);

        }
      }

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
    volatile aero::def::ParsedMessage_t* incoming_msg;

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

    void run_servos(aero::def::Commands_t* commands) {
      for (int i = 0; i < 8; i++) {
        if (aero::bit::read(commands->servos, i)) {
          Serial.print("[CMD] Opening Servo"); Serial.println(i);
          servos.open_servo(servos.m_servos[i]);
          if (i == 0) {
            digitalWrite(20, HIGH);
          }
          if (i >= 1 && i <= 5) {
            digitalWrite(21, HIGH);
          }
        }
      }

      for (int i = 8; i < 16; i++) {
        if (aero::bit::read(commands->servos, i)) {
          Serial.print("[CMD] Closing Servo"); Serial.println(i - 8);
          servos.close_servo(servos.m_servos[i - 8]);
          if ((i - 8) == 0) {
            digitalWrite(20, LOW);
          }
          if ((i - 8) >= 1 && (i - 8) <= 5) {
            digitalWrite(21, LOW);
          }
        }
      }
    };

    void run_commands(aero::def::Commands_t* commands) {
      if (commands->drop & OPEN_DOORS_MASK) {
        Serial.println("[CMD] Opening Doors");
        servos.actuate(DOOR);
      }
      if (commands->drop & PAYLOAD0_DROP_MASK) {
        Serial.println("[CMD] Dropping Payload0");
        servos.actuate(PAYLOAD0);
      }
      if (commands->drop & PAYLOAD1_DROP_MASK) {
        Serial.println("[CMD] Dropping Payload1");
        servos.actuate(PAYLOAD1);
      }
      if (commands->drop & PAYLOAD2_DROP_MASK) {
        Serial.println("[CMD] Dropping Payload2");
        servos.actuate(PAYLOAD2);
      }
      if (commands->drop & GLIDER0_DROP_MASK) {
        Serial.println("[CMD] Dropping Glider0");
        servos.actuate(GLIDER0);
      }
      if (commands->drop & GLIDER1_DROP_MASK) {
        Serial.println("[CMD] Dropping Glider1");
        servos.actuate(GLIDER1);
      }
      if (commands->drop & PAYLOAD0_RESET_MASK) {
        Serial.println("[CMD] Resetting Payload0");
        servos.reset(PAYLOAD0);
      }
      if (commands->drop & PAYLOAD1_RESET_MASK) {
        Serial.println("[CMD] Resetting Payload1");
        servos.reset(PAYLOAD1);
      }
      if (commands->drop & PAYLOAD2_RESET_MASK) {
        Serial.println("[CMD] Resetting Payload2");
        servos.reset(PAYLOAD2);
      }
      if (commands->drop & GLIDER0_RESET_MASK) {
        Serial.println("[CMD] Resetting Glider0");
        servos.reset(GLIDER0);
      }
      if (commands->drop & GLIDER1_RESET_MASK) {
        Serial.println("[CMD] Resetting Glider1");
        servos.reset(GLIDER1);
      }
      if (commands->drop & CLOSE_DOORS_MASK) {
        Serial.println("[CMD] Closing Doors");
        servos.reset(DOOR);
      }
    }
};
