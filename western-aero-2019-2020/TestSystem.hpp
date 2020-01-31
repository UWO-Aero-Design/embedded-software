/** \file TestSystem.hpp
 * @brief All testing class systems
 */

#include "System.hpp"

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*************************** SYSTEM FOR FULL TEST **************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
/*!
  @brief Strategy class for testing the entire system - verbose serial prints
*/
class TestSystem : public System {
public:
  // Description string
  static constexpr const char* DESCRIPTION = "Full System Test";

  /**
   * @brief System initialization
   *
   */
  bool init() override {
    return true;
  }

  /**
   * @brief Update system
   *
   */
  bool update() override {
    return true;
  }

private:

};

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* SYSTEM FOR TESTING SERIAL TRANSMITTING TO GROUND STATION WITH TEST DATA */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
/*!
  @brief Strategy class for testing serial dumps of comms (ground station)
*/
class txSerial : public System {

public:
    // Description of the system for printing
    static constexpr const char* DESCRIPTION = "Serial Transmit Test System";

    txSerial() {
        // Empty constructor
    }

    // Init method starts serial and builds test data
    bool init() override {

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
        return true;
    }

    bool update() override {
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
        return true;
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


/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/************ SYSTEM FOR DEMOING COMMS AT ZTR DESIGN REVIEW #1 *************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
/*!
  @brief Strategy class for use at ZTR Design Review #1
*/
class ZTRDemo1GndStation : public System {
public:
  // Description of the system for printing
  static constexpr const char* DESCRIPTION = "ZTR Demo #1 Ground Station Transmitter System";

  ZTRDemo1GndStation() {

  }

  bool init() override {

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
      return false;
    }
    else {
      Serial.println("LoRa radio init OK!");
      return true;
    }
  }

  bool update() override {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN] = {0};
    uint8_t len = sizeof(buf);

    if (radio.waitAvailableTimeout(3000)) {
      if (radio.recv(buf, &len)) {
        // Radio receive here
        digitalWrite(DEBUG_LED, HIGH);

        aero::def::RawMessage_t *tmp_msg = (aero::def::RawMessage_t *) &buf;

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
        return true;
      }
      else {
        Serial.println("recv failed");
        return false;
      }
    }
    else {
      Serial.println("No reply, is the plane running?");
      return false;
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
/*!
  @brief Strategy class for testing the pitot tube
*/
class PitotTubeDemo : public System {
public:
  // Description string
  static constexpr const char* DESCRIPTION = "Analog Pitot Tube Demo";

  /**
   * @brief System initialization that verifies sensor is connected properly
   *
   */
  bool init() override {

    // Check if pitot initialized properly
    bool init_result = pitot.init();

    if(!init_result) {
      Serial.println("Pitot tube seems to be not connected. Check wiring.");
      while(true);
      return false;
    }
    else {
      return true;
    }
  }

  /**
   * @brief Update system and update sensor value. Print out data
   *
   */
  bool update() override {
    // Check if pitot updated properly
    bool update_result = pitot.update();

    if(!update_result) {
      Serial.println("Pitot tube update failed");
      return false;
    } else {
      Serial.print("Differential pressure: ");
      Serial.print(pitot.pressure());
      Serial.print(" kPa and for message protocol: ");
      Serial.println(pitot.data().differential_pressure);
      return true;
    }

    delay(500);
  }

private:
  // Pin for the analog sensor
  static constexpr int PITOT_PIN = 23;

  // Pitot object
  PhidgetPitotTube pitot {aero::teensy35::P14_PWM};
};


/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/************* SYSTEM FOR TESTING ENVIRONMENT SENSOR CONNECTION ************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
/*!
  @brief Strategy class for testing the environemnt sensor
*/
class EnviroSensorDemo : public System {
public:
  // Description string
  static constexpr const char* DESCRIPTION = "Environment Sensor Demo";

  /**
   * @brief System initialization that verifies sensor is connected properly
   *
   */
  bool init() override {
    Wire.begin();

    // Check if environment sensor initialized properly
    bool init_result = enviro.init();

    if(!init_result) {
      Serial.println("Environment sensor seems to not be connected. Check wiring.");
      while(true);
      return false;
    }
    else {
      Serial.println("Environment sensor init complete.");
      return true;
    }
  }

  /**
   * @brief Update system and update sensor value. Print out data
   *
   */
  bool update() override {
    // Check if environment sensor updated properly
    bool update_result = enviro.update();

    if(!update_result) {
      Serial.println("Environment sensor update failed");
      return false;
    } else {
      Serial.print("Altitude: ");
      Serial.print(enviro.data().altitude / enviro.ALTITUDE_OFFSET);
      Serial.print(" [M] Temperature: ");
      Serial.print(enviro.data().temperature / enviro.TEMPERATURE_OFFSET);
      Serial.println(" [C]");
      return true;
    }

    delay(500);
  }

private:

  // Enviro object
  Mpl3115a2EnviroSensor enviro;
};

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************** SYSTEM FOR TESTING IMU SENSOR CONNECTION ****************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
/*!
  @brief Strategy class for testing the IMU sensor
*/
class IMUSensorDemo : public System {
public:
  // Description string
  static constexpr const char* DESCRIPTION = "IMU Sensor Demo";

  /**
   * @brief System initialization that verifies sensor is connected properly
   *
   */
  bool init() override {
    Wire.begin();

    // Check if environment sensor initialized properly
    bool init_result = m_imu.init();

    if(!init_result) {
      Serial.println("IMU sensor seems to not be connected. Check wiring.");
      while(true);
      return false;
    }
    else {
      Serial.println("IMU sensor init complete.");
      return true;
    }
  }

  /**
   * @brief Update system and update sensor value. Print out data
   *
   */
  bool update() override {
    // Check if environment sensor updated properly
    bool update_result = m_imu.update();

    if(!update_result) {
      Serial.println("IMU sensor update failed");
      return false;
    } else {
      Serial.print(m_imu.data().yaw);
      Serial.print(" ");
      Serial.print(m_imu.data().pitch);
      Serial.print(" ");
      Serial.println(m_imu.data().roll);
      return true;
    }

    delay(500);
  }

private:

  // IMU object
  ImuMpu9250 m_imu;
};

class AdafruitGPSDemo : public System {
public:
  // Description string
  static constexpr const char* DESCRIPTION = "Adafruit GPS Demo";
  AdafruitGPSDemo(){}
  /**
   * @brief System init
   *
   */
  bool init() override {

    // Check if pitot initialized properly
    bool init_result = gps.init();

    if(!init_result) {
      Serial.println("GPS failed hardware initialization. Check wiring.");
      while(true);
    }
  }

  /**
   * @brief System update
   *
   */
  bool update() override {
    // Use gps delay to let buffer fill up
    gps.delay(1000);
    bool update_result = gps.update();

    if(!update_result) {
      Serial.println("GPS update failed");
    } else {

        AdafruitGPS::TimeStamp timestamp = gps.timestamp();
        AdafruitGPS::Coord coord = gps.coord();
        int sats = gps.satellites();
        double speed = gps.speed();
        double alt = gps.altitude();
        double course = gps.angle();

        Serial.println("******************************");
        // Print connection
        Serial.println("Connection Status:");
        Serial.print("    Satellites: "); Serial.println(sats);

        // Print time stamp
        Serial.println("Date:");
        Serial.print("    (MM/DD/YY): "); Serial.print(timestamp.month);
        Serial.print("/"); Serial.print(timestamp.day);
        Serial.print("/"); Serial.print(timestamp.year);

        Serial.print("    Time:");
        Serial.print("    "); Serial.print(timestamp.hr);
        Serial.print(":"); Serial.print(timestamp.min);
        Serial.print(":"); Serial.println(timestamp.sec);

        // Print coordinate
        Serial.println("Coordinate:");
        Serial.print("    Lat: "); Serial.print(coord.lat, 7);
        Serial.print("    Lon: "); Serial.println(coord.lon, 7);

        // Print other data
        Serial.println("Misc Data:");
        Serial.print("    Speed (m/s): "); Serial.print(speed);
        Serial.print("    Altitude (m): "); Serial.print(alt);
        Serial.print("    Course (deg): "); Serial.println(course);

        // Print message data
        aero::def::GPS_t data = gps.data();

        Serial.println("Formatted Data:");
        Serial.print("    Connection: "); Serial.println(data.satellites);
        Serial.print("    Date: "); Serial.println(data.date);
        Serial.print("    Time: "); Serial.println(data.time);
        Serial.print("    Lat: "); Serial.print(data.lat);
        Serial.print("    Lon: "); Serial.println(data.lon);
        Serial.print("    Speed: "); Serial.println(data.speed);
        Serial.print("    Altitude: "); Serial.println(data.altitude);
        Serial.println("***********************************");
      }
  }

protected:
private:
  AdafruitGPS gps{&Serial4};
};

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/********************* SYSTEM FOR TESTING SERVO DRIVER *********************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
/*!
  @brief Strategy class for testing the onboard servo driver
*/
class ServoDriverDemo : public System {
public:
  // Description string
  static constexpr const char* DESCRIPTION = "Servo Driver Demo";

  /**
   * @brief System initialization that verifies sensor is connected properly
   *
   */
  bool init() override {
    Wire.begin();
    pinMode(20, OUTPUT);

    // Check if servo driver initialized properly
    bool init_result = servos.init();

    if(!init_result) {
      Serial.println("Servo driver seems to not be connected. Check wiring.");
      while(true);
      return false;
    }
    else {
      Serial.println("Servo driver init complete.");
      servos.close_all();
      return true;
    }
  }

  /**
   * @brief Update system and actuate servos
   *
   */
  bool update() override {

      Serial.println("Opening payload door.");
      digitalWrite(20, HIGH);
      servos.actuate(DOOR);
      delay(2000);

      Serial.println("Dropping payload0.");
      servos.actuate(PAYLOAD0);
      delay(300);

      Serial.println("Dropping payload1.");
      servos.actuate(PAYLOAD1);
      delay(300);

      Serial.println("Dropping payload2.");
      servos.actuate(PAYLOAD2);
      delay(300);

      Serial.println("Dropping gliders.");
      servos.actuate(GLIDER0);
      servos.actuate(GLIDER1);
      delay(500);

      Serial.println("Resetting drop mechanisms.");
      servos.reset(PAYLOAD0);
      servos.reset(PAYLOAD1);
      servos.reset(PAYLOAD2);
      servos.reset(GLIDER0);
      servos.reset(GLIDER1);
      delay(500);

      Serial.println("Closing payload door.\n");
      digitalWrite(20, LOW);
      servos.reset(DOOR);
      delay(2000);
  }

private:

  // Servo controller object
  ServoController servos;
};
