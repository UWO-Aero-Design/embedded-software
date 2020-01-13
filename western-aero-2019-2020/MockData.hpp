#pragma once

#include "src/aero-cpp-lib/include/Message.hpp"

struct MockData {
  
public:
  static aero::def::IMU_t test_imu(void) {
      aero::def::IMU_t imu {
          TEST_INT16,
          TEST_INT16,
          TEST_INT16,
          TEST_INT16,
          TEST_INT16,
          TEST_INT16,
          TEST_INT16,
          TEST_INT16,
          TEST_INT16,
          TEST_INT16,
          TEST_INT16,
          TEST_INT16
      };
  
      return imu;
  }
  
  static aero::def::Pitot_t test_pitot(void) {
      aero::def::Pitot_t pitot {
          TEST_INT16
      };
      return pitot;
  }
  
  static aero::def::GPS_t test_gps(void) {
      aero::def::GPS_t gps;
      // int32_t
      gps.lat = TEST_INT32;
      // int32_t
      gps.lon = TEST_INT32;
      // uint16_t
      gps.speed = TEST_UINT16;
      // uint8_t
      gps.satellites = TEST_UINT8;
      // uint16_t
      gps.altitude = TEST_UINT16;
      // uint32_t
      gps.time = TEST_UINT32;
      // uint32_t
      gps.date = TEST_UINT32;
  
      return gps;
  }
  
  static aero::def::Enviro_t test_enviro(void) {
      aero::def::Enviro_t enviro;
      // uint16_t for all
      enviro.altitude = TEST_UINT16;
      enviro.temperature = TEST_UINT16;
  
      return enviro;
  }
  
  static aero::def::Battery_t test_battery(void) {
      aero::def::Battery_t batt;
      // uint16_t for all
      batt.voltage = TEST_UINT16;
      batt.current = TEST_UINT16;
  
      return batt;
  }
  
  static aero::def::SystemConfig_t test_system(void) {
      aero::def::SystemConfig_t system;
      // Empty struct
      return system;
  }
  
  static aero::def::Status_t test_status(void) {
      aero::def::Status_t status;
      // int16_t
      status.rssi = TEST_INT16;
      // uint32_t
      status.state = TEST_UINT32;
  
      return status;
  }
  
  static aero::def::Servos_t test_servos(void) {
      aero::def::Servos_t servos;
      // All uint32
      servos.servo0 = TEST_UINT32;
      servos.servo1 = TEST_UINT32;
      servos.servo2 = TEST_UINT32;
      servos.servo3 = TEST_UINT32;
      servos.servo4 = TEST_UINT32;
      servos.servo5 = TEST_UINT32;
      servos.servo6 = TEST_UINT32;
      servos.servo7 = TEST_UINT32;
      servos.servo8 = TEST_UINT32;
      servos.servo9 = TEST_UINT32;
      servos.servo10 = TEST_UINT32;
      servos.servo11 = TEST_UINT32;
      servos.servo12 = TEST_UINT32;
      servos.servo13 = TEST_UINT32;
      servos.servo14 = TEST_UINT32;
      servos.servo15 = TEST_UINT32;
  
      return servos;
  }
  
  static aero::def::AirData_t test_airdata(void) {
      aero::def::AirData_t airdata;
      // All uint32
      airdata.ias = TEST_UINT32;
      airdata.eas = TEST_UINT32;
      airdata.tas = TEST_UINT32;
      airdata.agl = TEST_UINT32;
      airdata.pressure_alt = TEST_UINT32;
      airdata.msl = TEST_UINT32;
      airdata.density_alt = TEST_UINT32;
      airdata.approx_temp = TEST_UINT32;
      airdata.density = TEST_UINT32;
      
      return airdata;
  }
  
  static aero::def::Commands_t test_commands(void) {
      aero::def::Commands_t commands;
      // uint8_t
      commands.drop = TEST_UINT8;
      // uint16_t
      commands.servos = TEST_UINT16;
      // uint8_t
      commands.pitch = TEST_UINT8;
  
      return commands;
  }
  
  static aero::def::DropAlgo_t test_dropalgo(void) {
      aero::def::DropAlgo_t dropalgo;
      // int16_t
      dropalgo.heading = TEST_INT16;
      // uint16_t
      dropalgo.distance = TEST_UINT16;
  
      return dropalgo;
  }
  
protected:
private:
  static const uint8_t TEST_UINT8 = 123;
  static const int8_t TEST_INT8 = -123;
  
  static const uint16_t TEST_UINT16 = 12345;
  static const int16_t TEST_INT16 = -12345;
  
  static const uint32_t TEST_UINT32 = 1234567890;
  static const int32_t TEST_INT32 = -1234567890;

};
