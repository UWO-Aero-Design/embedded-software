/*
 * abstract class for defining how a system should be structured
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "Arduino.h"
#include "src/aero/aero-cpp-lib/include/Pins.hpp"
#include "src/aero/aero-cpp-lib/include/Data.hpp"
#include "src/aero/aero-cpp-lib/include/Message.hpp"
#include "ImuMpu9250.hpp"
#include "Rfm95w.hpp"
#include "MockData.hpp"

/**
 * @brief Astract class for defining how a system should be structured
 */
class System {
  public:
    System(){};
    virtual ~System(){};
    virtual void init() = 0;
    virtual void update() = 0;  
};

/*!
  @brief Implementation of a competition system
*/
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
    ImuMpu9250 *imu; /*!< An imu sensor */
    Rfm95w *radio; /*!< A radio */
    Message msg_handler; /*!< A message handler for creating messages */

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

/*!
  @brief Implementation of the test system
*/
class TestSystem : public System {
  public:
    TestSystem();
    ~TestSystem();
    /**  
     *  @brief Initialize the test system
     */
    void init() override;
    /**  
     *  @brief Update the test system
     */
    void update() override;

  private:
    String type = String("This is a test system");
    TestImuMpu9250 *imu;
  
};

class txSerial : public System {
public:
    txSerial() {
        // Empty constructor
    }

    void init() override {
        Serial.begin(115200);

        imu = MockData::test_imu();
        pitot = MockData::test_pitot();
        gps = MockData::test_gps();
        enviro = MockData::test_enviro();
        batt = MockData::test_battery();
        system_config = MockData::test_system();
        status_state = MockData::test_status();
        servos = MockData::test_servos();
        airdata = MockData::test_airdata();
        commands = MockData::test_commands();
        dropalgo = MockData::test_dropalgo();
    }

    void update() override {
        // Add to message buffer
        if(SEND_IMU) 
            msg_handler.add_imu(imu);
        if(SEND_PITOT)
            msg_handler.add_pitot(pitot);
        if(SEND_GPS)
            msg_handler.add_gps(gps);
        if(SEND_ENV)
            msg_handler.add_enviro(enviro);
        if(SEND_BATT)
            msg_handler.add_battery(batt);
        if(SEND_SYSTEM)
            msg_handler.add_config(system_config);
        if(SEND_STATUS)
            msg_handler.add_status(status_state);
        if(SEND_SERVOS)
            msg_handler.add_actuators(servos);
        if(SEND_AIRDATA)
            msg_handler.add_airdata(airdata);
        if(SEND_CMDS)
            msg_handler.add_cmds(commands);
        if(SEND_DROPALGO)
            msg_handler.add_drop(dropalgo);

        aero::def::RawMessage_t raw_msg = msg_handler.build(aero::def::ID::G1, aero::def::ID::G2);
        char *buf = (char *) &raw_msg;

        // Send message. Make sure to skip the part of the buffer that is empty
        for(int i = 0; i < sizeof(raw_msg); ++i) {
            // Skip empty parts of buffer
            if(i == raw_msg.length+6) {
                i = 256+6;
            }

            Serial.print((char)buf[i], HEX);
            Serial.print(' ');
        }
        
        Serial.print("\n");

        // Message every second
        delay(1000);
    }
protected:
private:
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

    aero::def::IMU_t imu;
    aero::def::Pitot_t pitot;
    aero::def::GPS_t gps;
    aero::def::Enviro_t enviro;
    aero::def::Battery_t batt;
    aero::def::SystemConfig_t system_config;
    aero::def::Status_t status_state;
    aero::def::Servos_t servos;
    aero::def::AirData_t airdata;
    aero::def::Commands_t commands;
    aero::def::DropAlgo_t dropalgo;

    aero::Message msg_handler;
};

class SystemSelect {
  public:
    SystemSelect();
    ~SystemSelect();

    /**  
     *  @brief The different valid systems the onboard code can boot into
     */
    enum SystemType {
      CompSystem_t = 0b00001111,
      TestSystem_t = 0b00000000,
      TesttxSerial_t = 0b00000001
    };
      
     /**
     * @brief Used to get a system object from a valid system type
     * 
     * @param type Type of system to return (SystemType)
     * @return System pointer of the correct type
    */
    static System *system_select(SystemType type) {
      switch(type) {
        case TestSystem_t:
          return new TestSystem();
          break;
        case CompSystem_t:
        default:
          return new CompSystem();
          break;
      }

      switch(type) {
        case TestSystem_t: {
          return new TestSystem();
        } break;

        case TesttxSerial_t: {
          return new txSerial();
        } break;

        case CompSystem_t: {
          return new CompSystem();
        } break;

        default: {
          return new CompSystem();
        } break;
      }
    }
    
    /**
     * @brief Used to get the name of a system from a valid system type. Purely for printing to the serial monitor.
     * 
     * @param type Type of system to return (SystemType)
     * @return String containing the name of the system
     */
    static String get_system_name(SystemType type) {
      switch(type) {
        case TestSystem_t:
          return "full test";
          break;
        case TesttxSerial_t: {
          return "Test Tx";
          break;
        }
        case CompSystem_t:
        default:
          return "competition";
          break;
    }
  }
};

#endif
