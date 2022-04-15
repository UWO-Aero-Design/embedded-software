/** \file CompSystem.hpp
   @brief Fully-fledged system that will be used during competiton
*/

#pragma once

#include "System.hpp"
#include "src/Message/pb_encode.h"
#include "src/Message/pb_decode.h"
#include "src/Message/message.pb.h"
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
      Message message = Message_init_zero;
      load_header(&message, Message_Location::Message_Location_GROUND_STATION, packet_number++, Message_Status::Message_Status_READY, 2);
      message.flight_stabilization = Message_FlightStabilization::Message_FlightStabilization_NONE;
    
//      bool imu_success = imu.update();
//      bool pitot_success = pitot.update();
      bool enviro_success = enviro.update();
//      bool gps_success = gps.update();

      // collect data from sensors
//      imu_data = imu.data();
//      pitot_data = pitot.data();
      if(enviro_success) {
        Enviro enviro_msg = Enviro_init_zero;
        enviro_data = enviro.data();
        enviro_msg.altitude = enviro_data.altitude;
        enviro_msg.pressure = enviro_data.pressure;
        enviro_msg.temperature = enviro_data.temperature;
        message.enviro = enviro_msg;
        message.has_enviro = true;
      }
//      gps_data = gps.data();

      send_stream = pb_ostream_from_buffer(send_buffer, sizeof(send_buffer));
      bool status = pb_encode(&send_stream, Message_fields, &message);

      if(status) {
        status = radio.send(send_buffer, send_stream.bytes_written);
        if(!status) {
          Serial.println("Error sending");
        }
        delay(100);
      }
      else {
        Serial.println("Error encoding");
      }

      if(millis() - last_print >= 500) {
        // fill print buffer with formatted text
//        sprintf(print_buffer, "IMU [YPR]: %-7.2f %-7.2f %-7.2f\tEnviro [A/T/P]: %-7.2f %-7.2f %-7.2f\tGPS [SAT]: %-2i",
//              imu_data.yaw, imu_data.pitch, imu_data.roll,
//              enviro_data.altitude, enviro_data.temperature, enviro_data.pressure , gps_data.satellites);
          sprintf(print_buffer, "Enviro [A/T/P]: %-7.2f %-7.2f %-7.2f", enviro_data.altitude, enviro_data.temperature, enviro_data.pressure);
        Serial.println(print_buffer);
//        algo.print();
        last_print = millis();
      }

      

      


//      return imu_success && pitot_success && enviro_success && gps_success;
  return true;
    }

  protected:

  private:
    long last_print = 0;
    long last_led_update = 0;
    uint32_t packet_number = 0;
    uint8_t send_buffer[128];
    pb_ostream_t send_stream;
  
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

    // Sensors
    ImuMpu9250 imu;
    PhidgetPitotTube pitot {aero::teensy35::P14_PWM};
    AdafruitBMP280EnviroSensor enviro;
    Radio_Rfm95w radio;
    AdafruitGPS gps {&Serial3};
    ServoController servos;
    
    bool gps_fix = false;
    uint8_t GPS_FIX_PIN = aero::teensy35::P16;
    DropAlgo algo = DropAlgo(28.084217, -81.965614);

    // bitmasks for Commands_t struct of ParsedMessage_t
    const uint8_t OPEN_DOORS_MASK      = 0b00000001;
    const uint8_t CLOSE_DOORS_MASK     = 0b00000010;
    const uint8_t GLIDER_DROP_MASK     = 0b00000100;
    const uint8_t WATER_DROP_MASK      = 0b00001000;
    const uint8_t HABITATS_DROP_MASK   = 0b00010000;

    void load_header(Message *msg, Message_Location recipient, uint32_t packet_number, Message_Status status, uint16_t rssi) {
      msg->sender = Message_Location::Message_Location_PLANE;
      msg->recipient = recipient;
      msg->packet_number = packet_number;
      msg->time = millis();
      msg->status = status;
      msg->rssi = rssi;
    }

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
