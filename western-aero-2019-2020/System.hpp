/*
 * abstract class for defining how a system should be structured
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "Arduino.h"
#include "src/aero-cpp-lib/include/Pins.hpp"
#include "src/aero-cpp-lib/include/Data.hpp"
#include "src/aero-cpp-lib/include/Message.hpp"
#include "ImuMpu9250.hpp"
#include "Rfm95w.hpp"
#include "MockData.hpp"
#include "PitotTube.hpp"
#include "src/Rfm95w/RH_RF95.h"

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
    int CompSystem::remove_msg_padding(RawMessage_t msg, char *new_buf);
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

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* SYSTEM FOR TESTING SERIAL TRANSMITTING TO GROUND STATION WITH TEST DATA */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
class txSerial : public System {
  
public:
    // Description of the system for printing
    static constexpr const char* DESCRIPTION = "Serial Transmit Test System";
    
    txSerial() {
        // Empty constructor
    }

    // Init method starts serial and builds test data
    void init() override {
        // Serial object initialization
        Serial.begin(115200);

        // Generating test data
        imu     = MockData::test_imu();
        pitot   = MockData::test_pitot();
        gps     = MockData::test_gps();
        enviro  = MockData::test_enviro();
        batt    = MockData::test_battery();
        system_config = MockData::test_system();
        status_state  = MockData::test_status();
        servos    = MockData::test_servos();
        airdata   = MockData::test_airdata();
        commands  = MockData::test_commands();
        dropalgo  = MockData::test_dropalgo();
    }

    void update() override {
        // Add to message buffer if configured to do so
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
    // None
  
private:
    // Flags to tell message builder whether or not to add certain data. If TRUE, will add data. Else, will not
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

    // Structs for data
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

    // Message handler
    aero::Message msg_handler;
};

class ZTRDemo1GndStation : public System {
public:
  // Description of the system for printing
  static constexpr const char* DESCRIPTION = "ZTR Demo #1 Ground Station Transmitter System";
    
  ZTRDemo1GndStation() {

  }

  void init() override {
    // Default baudrate required
    Serial.begin(115200);

    // Setup radio pins
    pinMode(DEBUG_LED, OUTPUT);
    pinMode(RFM95W_RST, OUTPUT);
    digitalWrite(DEBUG_LED, HIGH);
    delay(1000);
    digitalWrite(DEBUG_LED, LOW);

    // Reset radio
    digitalWrite(RFM95W_RST, LOW);
    delay(20);
    digitalWrite(RFM95W_RST, HIGH);
    delay(20);

    if(!radio.init()) {
      Serial.println("LoRa radio init failed");
      while(1);
    }
    else {
      Serial.println("LoRa radio init OK!");
    }
  }

  void update() override {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = {0};
    uint8_t len = sizeof(buf);

    if (radio.waitAvailableTimeout(3000)) {
      if (radio.recv(buf, &len)) {
        // Radio receive here
        digitalWrite(DEBUG_LED, HIGH);

        aero::def::RawMessage_t *tmp_msg = (RawMessage_t *) &buf;
        
        // Send message. Make sure to skip the part of the buffer that is empty
        for(int i = 0; i < 209; ++i) {
            // Skip empty parts
            if(i == tmp_msg->length+6) {
                i = 200+6;
            }
            
            Serial.print((char)buf[i], HEX);
            Serial.print(' ');
        }

        Serial.print("\n");

        digitalWrite(DEBUG_LED, LOW);
      }
      else {
        // Serial.println("recv failed");
      }
    }
    else {
      // Serial.println("No reply, is the plane running?");
    }
  }

private:
  // Teensy pin 13
  static const int DEBUG_LED = 13;
  // Radio pins
  static const int RFM95W_RST = 2;
  static const int RFM95W_INT = 9;
  static const int RFM95W_CS = 10;
  
  // Radio object to receive data
  RH_RF95 radio{RFM95W_CS, RFM95W_INT};
};

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/************* SYSTEM FOR TESTING ANALOG PITOT TUBE CONNECTION *************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
class PitotTubeDemo : public System {
public:
  // Description string
  static constexpr const char* DESCRIPTION = "Analog Pitot Tube Demo";

  /**
   * @brief System initialization that verifies sensor is connected properly
   * 
   */
  void init() override {
    Serial.begin(9600);
    // Delay so Serial has time to begin. Without it, any init/setup prints do not work
    delay(1000);

    // Check if pitot initialized properly
    bool init_result = pitot.init();

    if(!init_result) {
      Serial.println("Pitot tube seems to be not connected. Check wiring.");
      while(true);
    } 
  }

  /**
   * @brief Update system and update sensor value. Print out data
   * 
   */
  void update() override {
    // Check if pitot updated properly
    bool update_result = pitot.update();

    if(!update_result) {
      Serial.println("Pitot tube update failed");
    } else {
      Serial.print("Differential pressure: ");
      Serial.print(pitot.pressure());
      Serial.print(" kPa and for message protocol: ");
      Serial.println(pitot.data().differential_pressure);
    }

    delay(500);
  }

private:
  // Pin for the analog sensor
  static constexpr int PITOT_PIN = 23;

  // Pitot object
  PhidgetPitotTube pitot {aero::teensy35::A9_PWM};
};

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/********************* SYSTEM SELECT MECHANISM/FACTORY *********************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
class SystemSelect {
  public:
    /**  
     *  @brief The different valid systems the onboard code can boot into
     */
    enum Type {
      TestSystem_t    = 0b00000000, // System for testing system select
      TesttxSerial_t  = 0b00000001, // System for sending messages based on protocol over serial
      ZTRDemo1Gnd_t   = 0b00000010, // System for first ZTR target demo
      PitotDemo_t     = 0b00000100, // System for testing the analog phidget pitot tube
      CompSystem_t    = 0b00001111  // System for competition
    };
      
     /**
     * @brief Used to get a system object from a valid system type
     * 
     * @param type Type of system to return (SystemType)
     * @return System pointer of the correct type
    */
    static System *select(Type type) {
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

        case ZTRDemo1Gnd_t: {
          return new ZTRDemo1GndStation();
        } break;

        case PitotDemo_t: {
          return new PitotTubeDemo();
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
    static String get_description(Type type) {
      switch(type) {
        case TestSystem_t: {
          return "Test system";
        } break;
        
        case TesttxSerial_t: {
          return txSerial::DESCRIPTION;
        } break;

        case ZTRDemo1Gnd_t: {
          return ZTRDemo1GndStation::DESCRIPTION;
        } break;

        case PitotDemo_t: {
          return PitotTubeDemo::DESCRIPTION;
        } break;
        
        case CompSystem_t:
        default:
          return "competition";
          break;
    }
  }
};

#endif
