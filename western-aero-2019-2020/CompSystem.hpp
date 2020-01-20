#pragma once

#include "System.hpp"
#include "Wire.h"
#include "src/aero-cpp-lib/include/Data.hpp"
#include "src/aero-cpp-lib/include/Message.hpp"


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
    void init() override {
        // Serial object initialization
        Serial.begin(9600);
        if(imu.init()) {
          Serial.println("IMU online.");
        }
        else {
          Serial.println("Error connecting to IMU.");
        }
        if(enviro.init()) {
          Serial.println("Environment sensor online.");
        }
        else {
          Serial.println("Error connecting to environment sensor.");
        }
        if(pitot.init()) {
          Serial.println("Pitot tube online.");
        }
        else {
          Serial.println("Error connecting to pitot tube.");
        }
        if(radio.init()) {
          Serial.println("Radio online.");
        }
        else {
          Serial.println("Error connecting to radio.");
        }
        if(servos.init()) {
          Serial.println("Servo controller online.");
        }
        else {
          Serial.println("Error connecting to servo controller.");
        }
    }

    void update() override {
      imu.update();
      pitot.update();
      enviro.update();

      Serial.print("IMU [YPR] - ");
      Serial.print(imu.data().yaw, 2);
      Serial.print(" ");
      Serial.print(imu.data().pitch, 2);
      Serial.print(" ");
      Serial.print(imu.data().roll, 2);
      Serial.print(" Pitot [DP] - ");
      Serial.print(pitot.data().differential_pressure, 2);
      Serial.print(" Enviro [A/T] - ");
      Serial.print(enviro.data().altitude);
      Serial.print(" ");
      Serial.print(enviro.data().temperature);

      Serial.println();
      delay(100);
    }
    
protected:
  
private:

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
