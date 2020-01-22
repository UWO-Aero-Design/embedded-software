#pragma once

#include "System.hpp"
#include <stdio.h>


/*!
  @brief Implementation of a competition system
*/
/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************** FULL SYSTEM USED FOR COMPETITION  ********************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
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
      

      delay(100);
      return imu_success && pitot_success && enviro_success;
    }
    
protected:
  
private:
    // holds the nicely formatted string for printing
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
    Rfm95w radio;
};
