/** \file TestSystem.hpp
 * @brief All testing class systems
 */

#include "System.hpp"

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
      Serial.print("Calibrating sensor...");
      enviro.calibrate();
      Serial.println("Done.");
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
      Serial.print("[M] Temperature: ");
      Serial.print(enviro.data().temperature / enviro.STRUCT_TEMPERATURE_OFFSET);
      Serial.print(" [C] ");
      Serial.print(enviro.data().pressure / enviro.STRUCT_PRESSURE_OFFSET);
      Serial.print(" [Pa] ");
      Serial.print(" Altitude: ");
      Serial.print(enviro.data().altitude / enviro.STRUCT_ALTITUDE_OFFSET);
      Serial.print(" => ");
      Serial.println(enviro.data().altitude / enviro.STRUCT_ALTITUDE_OFFSET) - enviro.offset();;
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
/***************** SYSTEM FOR TESTING GPS SENSOR CONNECTION ****************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
/*!
  @brief Strategy class for testing the GPS sensor
*/
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
      delay(500);
  }

protected:
private:
  #if defined(GROUND_STATION)
    AdafruitGPS gps{&Serial1};  // Temp, does not mean anything
  #else
    AdafruitGPS gps{&Serial3};
  #endif
};

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/****************** SYSTEM FOR TESTING RADIO TRANSMISSION ******************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
class RadioClientDemo : public System {
public:

  // Description string
  static constexpr const char* DESCRIPTION = "Radio Client (Transmission) Demo";

  /**
   * @brief Initialize client test object
   */
  bool init() override {
    Serial.begin(115200);
    delay(2000);

    bool success = radio.init();

    if(!success) {
      Serial.println("Init failed");
      return false;
    }
    return true;

  }

  /**
   * @brief Update client system
   * @details Send message every second and print when a message is received
   */
  bool update() override {
    RawMessage_t client_message = message_handler.build(aero::def::ID::Gnd, aero::def::ID::Plane, true);

    ParsedMessage_t* server_response = radio.send(client_message);

    if(server_response != NULL) {

      Serial.println("Response received");

      /* Parse message here... */
    }

    delay(1000);
    return true;
  }

private:
  int cs_pin =  aero::teensy35::P10_PWM;
  int rst_pin = aero::teensy35::P34;
  int int_pin = aero::teensy35::P31;

  RFM95WClient radio{ cs_pin, rst_pin, int_pin };
  aero::Message message_handler;
};

/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************* SYSTEM FOR TESTING RADIO RECEPTION ********************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
class RadioServerDemo : public System {
public:

  // Description string
  static constexpr const char* DESCRIPTION = "Radio Server (Reception) Demo";

  /**
   * @brief Initialize server test object
   */
  bool init() override {
    Serial.begin(9600);
    delay(2000);

    bool success = radio.init();

    if(!success) {
      Serial.println("Radio init failed");
      return false;
    }
    else {
      return true;
    }
  }

  /**
   * @brief Update server system
   * @details Wait for new message from client and respond when one is received
   *
   */
  bool update() override {
    RawMessage_t server_response = message_handler.build(aero::def::ID::Plane, aero::def::ID::Gnd, true);

    aero::def::ParsedMessage_t* client_message = NULL;

    if(client_message != NULL) {
      Serial.println("Message received and responded too");

      if(client_message->pitot() != NULL) {
        Serial.println("Pitot Tube:");
        Serial.print("\tDifferential pressure: "); Serial.println(client_message->pitot()->differential_pressure);
      }

      if(client_message->imu() != NULL) {
        Serial.println("IMU:");
        Serial.print("\tAX: ");    Serial.println(client_message->imu()->ax);
        Serial.print("\tAY: ");    Serial.println(client_message->imu()->ay);
        Serial.print("\tAZ: ");    Serial.println(client_message->imu()->az);
        Serial.print("\tGX: ");    Serial.println(client_message->imu()->gx);
        Serial.print("\tGY: ");    Serial.println(client_message->imu()->gy);
        Serial.print("\tGZ: ");    Serial.println(client_message->imu()->gz);
        Serial.print("\tMX: ");    Serial.println(client_message->imu()->mx);
        Serial.print("\tMY: ");    Serial.println(client_message->imu()->my);
        Serial.print("\tMZ: ");    Serial.println(client_message->imu()->mz);
        Serial.print("\tYaw: ");   Serial.println(client_message->imu()->yaw);
        Serial.print("\tPitch: "); Serial.println(client_message->imu()->pitch);
        Serial.print("\tRoll: ");  Serial.println(client_message->imu()->roll);
      }

      if(client_message->gps() != NULL) {
        Serial.println("GPS:");
        Serial.print("\tLat: ");    Serial.println(client_message->gps()->lat);
        Serial.print("\tLon: ");    Serial.println(client_message->gps()->lon);
        Serial.print("\tSpeed: ");    Serial.println(client_message->gps()->speed);
        Serial.print("\tSats: ");    Serial.println(client_message->gps()->satellites);
        Serial.print("\tAlt: ");    Serial.println(client_message->gps()->altitude);
        Serial.print("\tTime: ");    Serial.println(client_message->gps()->time);
        Serial.print("\tDate: ");    Serial.println(client_message->gps()->date);
      }

      if(client_message->enviro() != NULL) {
        Serial.println("Enviro:");
        Serial.print("\tAlt: ");    Serial.println(client_message->enviro()->altitude);
        Serial.print("\tTemp: ");    Serial.println(client_message->enviro()->temperature);
        Serial.print("\tPressure: ");    Serial.println(client_message->enviro()->pressure);
      }

      if(client_message->battery() != NULL) {
        Serial.println("Battery:");
        Serial.print("\tVoltage: ");    Serial.println(client_message->battery()->voltage);
        Serial.print("\tCurrent: ");    Serial.println(client_message->battery()->current);
      }

      if(client_message->config() != NULL) {
        /* Currently an empty struct; nothing to read */
      }

      if(client_message->status() != NULL) {
        Serial.println("Status:");
        Serial.print("\tRSSI: ");    Serial.println(client_message->status()->rssi);
        Serial.print("\tState: ");    Serial.println(client_message->status()->state);
      }

      if(client_message->servos() != NULL) {
        Serial.println("Servos:");
        Serial.print("\t0: ");    Serial.println(client_message->servos()->servo0);
        Serial.print("\t1: ");    Serial.println(client_message->servos()->servo1);
        Serial.print("\t2: ");    Serial.println(client_message->servos()->servo2);
        Serial.print("\t3: ");    Serial.println(client_message->servos()->servo3);
        Serial.print("\t4: ");    Serial.println(client_message->servos()->servo4);
        Serial.print("\t5: ");    Serial.println(client_message->servos()->servo5);
        Serial.print("\t6: ");    Serial.println(client_message->servos()->servo6);
        Serial.print("\t7: ");    Serial.println(client_message->servos()->servo7);
        Serial.print("\t8: ");    Serial.println(client_message->servos()->servo8);
        Serial.print("\t9: ");    Serial.println(client_message->servos()->servo9);
        Serial.print("\t10: ");    Serial.println(client_message->servos()->servo10);
        Serial.print("\t11: ");    Serial.println(client_message->servos()->servo11);
        Serial.print("\t12: ");    Serial.println(client_message->servos()->servo12);
        Serial.print("\t13: ");    Serial.println(client_message->servos()->servo13);
        Serial.print("\t14: ");    Serial.println(client_message->servos()->servo14);
        Serial.print("\t15: ");    Serial.println(client_message->servos()->servo15);
      }

      if(client_message->air_data() != NULL) {
        Serial.println("Air Data:");
        Serial.print("\tIAS: ");    Serial.println(client_message->air_data()->ias);
        Serial.print("\tEAS: ");    Serial.println(client_message->air_data()->eas);
        Serial.print("\tTAS: ");    Serial.println(client_message->air_data()->tas);
        Serial.print("\tAGL: ");    Serial.println(client_message->air_data()->agl);
        Serial.print("\tPressure Alt: ");    Serial.println(client_message->air_data()->pressure_alt);
        Serial.print("\tMSL: ");    Serial.println(client_message->air_data()->msl);
        Serial.print("\tDensity Alt: ");    Serial.println(client_message->air_data()->density_alt);
        Serial.print("\tApprox Temp: ");    Serial.println(client_message->air_data()->approx_temp);
        Serial.print("\tDensity: ");    Serial.println(client_message->air_data()->density);
      }

      if(client_message->cmds() != NULL) {
        Serial.println("Commands:");
        Serial.print("\tDrop: ");    Serial.println(client_message->cmds()->drop);
        Serial.print("\tServos: ");    Serial.println(client_message->cmds()->servos);
        Serial.print("\tPitch: ");    Serial.println(client_message->cmds()->pitch);
      }

      if(client_message->drop_algo() != NULL) {
        Serial.println("Drop Algorithm:");
        Serial.print("\tHeading: ");    Serial.println(client_message->drop_algo()->heading);
        Serial.print("\tDistance: ");    Serial.println(client_message->drop_algo()->distance);
      }
    }

    delay(100);
    return true;
  }

private:
  int cs_pin =  aero::teensy35::P10_PWM;
  int rst_pin = aero::teensy35::P34;
  int int_pin = aero::teensy35::P31;

  RFM95WServer radio{ cs_pin, rst_pin, int_pin };
  aero::Message message_handler;
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
      Serial.print(m_imu.data().yaw / 100.0);
      Serial.print(" ");
      Serial.print(m_imu.data().pitch / 100.0);
      Serial.print(" ");
      Serial.println(m_imu.data().roll / 100.0);
      return true;
    }

    delay(500);
  }

private:

  // IMU object
  ImuMpu9250 m_imu;
};

