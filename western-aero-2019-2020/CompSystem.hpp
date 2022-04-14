/** \file CompSystem.hpp
   @brief Fully-fledged system that will be used during competiton
*/

#pragma once

#include "System.hpp"
#include "src/aero-cpp-lib/include/Utility.hpp"
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
//      if (imu.init()) {
//        Serial.println("IMU online.");
//      }
//      else {
//        Serial.println("Error connecting to IMU.");
//        is_success = false;
//      }
      if (enviro.init()) {
        Serial.println("Environment sensor online.");
      }
      else {
        Serial.println("Error connecting to environment sensor.");
        is_success = false;
      }
//      if (pitot.init()) {
//        Serial.println("Pitot tube online.");
//      }
//      else {
//        Serial.println("Error connecting to pitot tube.");
//        is_success = false;
//      }
      if (radio.init()) {
        Serial.println("Radio online.");
      }
      else {
        Serial.println("Error connecting to radio.");
        is_success = false;
      }
//      if (servos.init()) {
//        Serial.println("Servo controller online.");
//      }
//      else {
//        Serial.println("Error connecting to servo controller.");
//        is_success = false;
//      }
//      if (gps.init()) {
//        Serial.println("GPS online.");
//      }
//      else {
//        Serial.println("Error connecting to GPS.");
//        is_success = false;
//      }
      
//      Serial.println("\n");
//
      Serial.print("Calibrating Enviro...");
      if(enviro.calibrate()) {
        Serial.println("Done.");
      }
      else {
        Serial.println("Error calibrating the enviro.");
        is_success = false;
      }

//      Serial.print("Calibrating IMU...");
//      imu.calibrate();
//      Serial.println("Done.");

//      digitalWrite(20, LOW);
      
      return is_success;
return true;
    }

    bool update() override {
//      bool imu_success = imu.update();
//      bool pitot_success = pitot.update();
      bool enviro_success = enviro.update();
//      bool gps_success = gps.update();

      // collect data from sensors
//      imu_data = imu.data();
//      pitot_data = pitot.data();
      enviro_data = enviro.data();
//      gps_data = gps.data();
      
            
//      Serial.println(print_buffer);

      // Add to message buffer if configured to do so
//      if(SEND_IMU)       msg_handler.add_imu(imu_data);
//      if(SEND_PITOT)     msg_handler.add_pitot(pitot_data);
//      if(SEND_GPS)       msg_handler.add_gps(gps_data);
//      if(SEND_ENV)       msg_handler.add_enviro(enviro_data);
//      if(SEND_BATT)      msg_handler.add_battery(batt_data);
//      if(SEND_SYSTEM)    msg_handler.add_config(system_config_data);
//      if(SEND_STATUS)    msg_handler.add_status(status_state_data);
//      if(SEND_SERVOS)    msg_handler.add_actuators(servos_data);
//      if(SEND_AIRDATA)   msg_handler.add_airdata(airdata_data);
//      if(SEND_CMDS)      msg_handler.add_cmds(commands_data);
//      if(SEND_DROPALGO)  msg_handler.add_drop(dropalgo_data);

      aero::def::RawMessage_t raw_msg = msg_handler.build(aero::def::ID::Plane, aero::def::ID::Gnd);
      char *buf = (char *) &raw_msg;

      // Send message. Make sure to skip the part of the buffer that is empty
      for(int i = 0; i < sizeof(raw_msg); ++i) {
        if(i == raw_msg.length+6) {
          i = 256+6;
        }
      }
    


//      algo.set_height(enviro_data.altitude / Mpl3115a2EnviroSensor::STRUCT_ALTITUDE_OFFSET);
//      algo.set_coords(gps_data.lat / AdafruitGPS::LAT_SCALAR, gps_data.lon / AdafruitGPS::LON_SCALAR);
//      algo.update();

      if(millis() - last_print >= 500) {
        // fill print buffer with formatted text
        sprintf(print_buffer, "IMU [YPR]: %-7.2f %-7.2f %-7.2f\tPitot: %4i\tEnviro [A/T/P]: %-7.2f %-7.2f %-7.2f\tGPS [SAT]: %-2i",
              imu_data.yaw / 100.0, imu_data.pitch / 100.0, imu_data.roll / 100.0,
              pitot_data.differential_pressure,
              (enviro_data.altitude) / 100.0, enviro_data.temperature / 100.0, enviro_data.pressure / 100.0 , gps_data.satellites);
        Serial.println(print_buffer);
//        algo.print();
        last_print = millis();
      }


//      msg_handler.add_imu(imu_data);
//      msg_handler.add_pitot(pitot_data);
      msg_handler.add_enviro(enviro_data);
//      msg_handler.add_gps(gps_data);

      RawMessage_t response_to_gnd = msg_handler.build(aero::def::ID::Plane, aero::def::ID::Gnd, true);
      radio.respond(response_to_gnd);


//      aero::def::ParsedMessage_t* radio_recv = radio.receive();

      // Receive the incoming message
//      if ( radio_recv != NULL ) {
//        if (radio_recv->m_to == aero::def::ID::Plane) {
//          digitalWrite(22, HIGH);
//          Serial.println("Message received from ground station");
//          if (radio_recv->cmds() != NULL) {
//           run_servos(radio_recv->cmds());
//           run_commands(radio_recv->cmds());
//         }
//         radio.respond(response_to_gnd);
//        }
//      }

//      if(millis() - last_led_update >= 1000) {
//        digitalWrite(21, LOW);
//      }

//      return imu_success && pitot_success && enviro_success && gps_success;
  return true;
    }

  protected:

  private:
    long last_print = 0;
    long last_led_update = 0;
  
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

    // Flags to tell message builder whether or not to add certain data
    const static bool SEND_IMU = true;
    const static bool SEND_PITOT = true;
    const static bool SEND_GPS = true;
    const static bool SEND_ENV = true;
    const static bool SEND_BATT = true;
    const static bool SEND_SYSTEM = true;
    const static bool SEND_STATUS = true;
    const static bool SEND_SERVOS = true;
    const static bool SEND_AIRDATA = true;
    const static bool SEND_CMDS = true;
    const static bool SEND_DROPALGO = true;

    // Message handler
    aero::Message msg_handler;

    // Servo controller
    ServoController servos;

    // Sensors
    ImuMpu9250 imu;
    PhidgetPitotTube pitot {aero::teensy35::P14_PWM};
    AdafruitBMP280EnviroSensor enviro;
    RFM95WServer radio{ aero::teensy35::P10_PWM, aero::teensy35::P34, aero::teensy35::P31 };
#ifdef GROUND_STATION
    AdafruitGPS gps {&Serial1};
#else
    AdafruitGPS gps {&Serial3};
#endif
    bool gps_fix = false;
    uint8_t GPS_FIX_PIN = aero::teensy35::P16;
    DropAlgo algo = DropAlgo(28.084217, -81.965614);

    // bitmasks for Commands_t struct of ParsedMessage_t
    const uint8_t OPEN_DOORS_MASK      = 0b00000001;
    const uint8_t CLOSE_DOORS_MASK     = 0b00000010;
    const uint8_t GLIDER_DROP_MASK     = 0b00000100;
    const uint8_t WATER_DROP_MASK      = 0b00001000;
    const uint8_t HABITATS_DROP_MASK   = 0b00010000;

    void run_servos(aero::def::Commands_t* commands) {
      for (int i = 0; i < 8; i++) {
        if (aero::bit::read(commands->servos, i)) {
        
          Serial.print("[CMD] Opening Servo"); Serial.println(i + 1);
          servos.open_servo(servos.m_servos[i]);
          digitalWrite(21, HIGH);
          last_led_update = millis();
        }
      }

      for (int i = 8; i < 16; i++) {
        if (aero::bit::read(commands->servos, i)) {
          Serial.print("[CMD] Closing Servo"); Serial.println(i - 8 + 1);
          servos.close_servo(servos.m_servos[i - 8]);
          digitalWrite(21, HIGH);
          last_led_update = millis();
        }
      }
    };

    void run_commands(aero::def::Commands_t* commands) {
      if (commands->drop & OPEN_DOORS_MASK) {
        Serial.println("[CMD] Opening Doors");
        servos.actuate(DOOR0);
        servos.actuate(DOOR1);
        digitalWrite(21, HIGH);
        last_led_update = millis();
      }
      if (commands->drop & GLIDER_DROP_MASK) {
        Serial.println("[CMD] Dropping Gliders");
        servos.actuate(GLIDER0);
        servos.actuate(GLIDER1);
        digitalWrite(21, HIGH);
        last_led_update = millis();
      }
      if (commands->drop & WATER_DROP_MASK) {
        Serial.println("[CMD] Dropping Water");
        servos.actuate(PAYLOAD0);
        servos.actuate(PAYLOAD1);
        servos.actuate(PAYLOAD2);
        digitalWrite(21, HIGH);
        last_led_update = millis();
      }
      if (commands->drop & HABITATS_DROP_MASK) {
        Serial.println("[CMD] Dropping Habitats");
        servos.actuate(PAYLOAD0);
        servos.actuate(PAYLOAD1);
        servos.actuate(PAYLOAD2);
        digitalWrite(21, HIGH);
        last_led_update = millis();
      }
      if (commands->drop & CLOSE_DOORS_MASK) {
        Serial.println("[CMD] Closing Doors");
        servos.reset(DOOR0);
        servos.reset(DOOR1);
        digitalWrite(21, HIGH);
        last_led_update = millis();
      }
    }
};
