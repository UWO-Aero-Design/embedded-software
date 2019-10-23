/*
 * abstract class for defining how a system should be structured
 */

#pragma once

#include "Arduino.h"
#include "src/aero-cpp-lib/include/Pins.hpp"
#include "src/aero-cpp-lib/include/Data.hpp"
#include "src/aero-cpp-lib/include/Message.hpp"
#include "ImuMpu9250.h"
#include "Rfm95w.h"


// abstract class for defining how a system should be structured
class System {
  public:
    System(){}
    virtual ~System(){}
    virtual void init() = 0;
    virtual void update() = 0;  
};

// implementation of the competiton system
class CompSystem : public System {
  public:
    CompSystem();
    ~CompSystem();
    void init() override;
    void update() override;

  private:
    const String type = String("This is a competition system");
    void CompSystem::remove_msg_padding(RawMessage_t msg, char *new_buf);
    void build_message();
    void temp_fill_data();
    ImuMpu9250 *imu;
    Rfm95w *radio;
    Message msg_handler;

    // data containers - to be removed upon sensor implementation
    Pitot_t pitot_data;
    Enviro_t enviro_data;
    IMU_t imu_data;
    GPS_t gps_data;
    Battery_t battery_data;
    SystemConfig_t system_config_data;
    Status_t status_data;
    Servos_t servos_data;
    AirData_t air_data;
    Commands_t commands_data;
    DropAlgo_t drop_algo_data;
  
};

// implementation of the test system
class TestSystem : public System {
  public:
    TestSystem();
    ~TestSystem();
    void init() override;
    void update() override;

  private:
    String type = String("This is a test system");
    TestImuMpu9250 *imu;
  
};

// factory-styled pattern for creating a system
class SystemSelect {
  public:
    SystemSelect();
    ~SystemSelect();
    enum SystemType { CompSystem_t = 0b00001111, TestSystem_t = 0b00000000 };
    static System *system_select(uint8_t type) {
      switch(type) {
        case TestSystem_t:
          return new TestSystem();
          break;
        case CompSystem_t:
        default:
          return new CompSystem();
          break;
      }
    };
    static String get_system_name(uint8_t type) {
      switch(type) {
        switch(type) {
        case TestSystem_t:
          return "full test";
          break;
        case CompSystem_t:
        default:
          return "competition";
          break;
      }
    }
  }
};
