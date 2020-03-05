/** \file CompSystem.hpp
   @brief Fully-fledged system that will be used during competiton
*/

#pragma once

#include "System.hpp"
#include "src/aero-cpp-lib/include/Pins.hpp"
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
    }

    ~CompSystem() {
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
      if (gps.init()) {
        Serial.println("GPS online.");
      }
      else {
        Serial.println("Error connecting to GPS.");
        is_success = false;
      }

      Serial.println("\n");

      Serial.print("Calibrating Enviro...");
      //enviro.calibrate();
      Serial.println("Done.");

      Serial.print("Calibrating IMU...");
      imu.calibrate();
      Serial.println("Done.");

      
      
      return is_success;
    }

    bool update() override {
      bool imu_success = imu.update();
      bool pitot_success = pitot.update();
      bool enviro_success = enviro.update();
      bool gps_success = gps.update();

      // collect data from sensors
      imu_data = imu.data();
      pitot_data = pitot.data();
      enviro_data = enviro.data();
      gps_data = gps.data();


      // fill print buffer with formatted text
      sprintf(print_buffer, "IMU [YPR]: %-7.2f %-7.2f %-7.2f\tPitot: %4i\tEnviro [A/T]: %-7.2f %-7.2f\tGPS [SAT]: %-2i",
              imu_data.yaw / 100.0, imu_data.pitch / 100.0, imu_data.roll / 100.0,
              pitot_data.differential_pressure,
              enviro_data.altitude / 100.0, enviro_data.temperature / 100.0, gps_data.satellites);
      if(millis() - last_print >= 500) {
        Serial.println(print_buffer);
        last_print = millis();
      }


      msg_handler.add_imu(imu_data);
      msg_handler.add_pitot(pitot_data);
      msg_handler.add_enviro(enviro_data);
      msg_handler.add_gps(gps_data);

      RawMessage_t response_to_gnd = msg_handler.build(aero::def::ID::Plane, aero::def::ID::Gnd, true);


      aero::def::ParsedMessage_t* radio_recv = radio.receive();
      digitalWrite(22, LOW);

      // Receive the incoming message
      if ( radio_recv != NULL ) {

        if (radio_recv->m_to == aero::def::ID::Plane) {
          digitalWrite(22, HIGH);
          Serial.println("Message received from ground station");
          if (radio_recv->cmds() != NULL) {
           run_servos(radio_recv->cmds());
           run_commands(radio_recv->cmds());
         }

         radio.respond(response_to_gnd);
        }
      }

      return imu_success && pitot_success && enviro_success && gps_success;
    }

  protected:

  private:
    long last_print = 0;
  
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
#ifdef GROUND_STATION
    AdafruitGPS gps {&Serial1};
#else
    AdafruitGPS gps {&Serial3};
#endif
    bool gps_fix = false;
    uint8_t GPS_FIX_PIN = aero::teensy35::P16;

    // bitmasks for Commands_t struct of ParsedMessage_t
    const uint8_t OPEN_DOORS_MASK      = 0;
    const uint8_t CLOSE_DOORS_MASK     = 1;
    const uint8_t GLIDER_DROP_MASK     = 2;
    const uint8_t WATER_DROP_MASK      = 3;
    const uint8_t HABITATS_DROP_MASK   = 4;

    void run_servos(aero::def::Commands_t* commands) {
      for (int i = 0; i < 8; i++) {
        if (aero::bit::read(commands->servos, i)) {
          Serial.print("[CMD] Opening Servo"); Serial.println(i + 1);
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
          Serial.print("[CMD] Closing Servo"); Serial.println(i - 8 + 1);
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
        servos.actuate(DOOR0);
        servos.actuate(DOOR1);
      }
      if (commands->drop & GLIDER_DROP_MASK) {
        Serial.println("[CMD] Dropping Gliders");
        servos.actuate(GLIDER0);
        servos.actuate(GLIDER1);
      }
      if (commands->drop & WATER_DROP_MASK) {
        Serial.println("[CMD] Dropping Water");
        servos.actuate(PAYLOAD1);
      }
      if (commands->drop & HABITATS_DROP_MASK) {
        Serial.println("[CMD] Dropping Habitats");
        servos.actuate(PAYLOAD0);
        servos.actuate(PAYLOAD2);
      }
      if (commands->drop & CLOSE_DOORS_MASK) {
        Serial.println("[CMD] Closing Doors");
        servos.reset(DOOR0);
        servos.reset(DOOR1);
      }
    }
};
