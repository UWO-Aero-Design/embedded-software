/** \file GPS.hpp
 * @brief All code relating to the GPS sensor
 */

#pragma once

// Sensor interface
#include "src/aero-cpp-lib/include/Sensors.hpp"
#include "src/aero-cpp-lib/include/Utility.hpp"

#include "Arduino.h"
#include "src/Adafruit_GPS/Adafruit_GPS.h"

#define GPSSerial Serial2

/**
 * @brief AdafruitGPS adapter class 
 * @details Communication to the sensor is handled via serial and parsing using TinyGPS
 */
class AdafruitGPS : public aero::sensor::GPS {
public: 

    /**
     * @brief GMT Time Stamp from GPS 
     */
    struct TimeStamp {
        byte hr;                                     
        byte min;                                   
        byte sec;                                  
        byte msec;                            
        int year;                                     
        byte month;                                    
        byte day;
        
        /**
         * @brief Extract a timestamp struct from date, time values
         * 
         * @param date byte field representing date
         * @param time byte field representing time
         * @return TimeStamp timestamp based on date/time encoded values
         */
        static TimeStamp extract(uint32_t date, uint32_t time) {
            TimeStamp ts;

            ts.day = (date & 0x000000ff);
            ts.month = (date & 0x0000ff00) >> 8;
            ts.year = (date & 0x00ff0000) >> 16;

            ts.sec = (time & 0x000000ff);
            ts.min = (time & 0x0000ff00) >> 8;
            ts.hr = (time & 0x00ff0000) >> 16;
        }                                      
    };

    /**
     * @brief GPS transmission statistics
     */
    struct Stats {
      unsigned long chars;
      unsigned short seqs;
      unsigned short fails;
    };

    /**
     * @brief GPS coordinates in decimal degrees
     */
    struct Coord {
        double lat;
        double lon;
    };

    // This class requires a hardware serial port. Therefore, do not allow default constructor
    AdafruitGPS() = delete;

    /**
     * @brief Construct a new Adafruit GPS object
     * 
     * @param port hardware serial port that the gps is connected too
     */
    AdafruitGPS(HardwareSerial *port) {
      this->port = port;
      gps = Adafruit_GPS(&GPSSerial);
      m_data.satellites = 0;
      gps.fix = false;
    }

    /**
     * @brief Initialize the GPS module
     * 
     * @return true If update succeeded
     * @return false If update failed
     */
    bool init() override {
        gps.begin(GPS_BAUD_RATE);
        
        gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // The data we are requesting
        gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
      
        // don't request updates on antenna status
        gps.sendCommand(PGCMD_NOANTENNA);
        
        return true;
    }

    long last_update = 0;

    /**
     * @brief Grabs data from the GPS module and updates internal variables
     * 
     * @return true If update succeeded
     * @return false If update failed
     */
    bool update() override {
    char c = gps.read();
    if (gps.newNMEAreceived()) {
      if (!gps.parse(gps.lastNMEA())) {
        return true; // we can fail to parse a sentence in which case we should just wait for another
      }
      else {
        m_is_new_data = true;
      }
    }
    
    m_data.fix = gps.fix;
    // time
    // date
    // HDOP
    // quality
      
    if(m_is_new_data) {
      if (gps.fix) {
//        Serial.print("Location: ");
//      Serial.print(gps.latitude, 4); Serial.print(gps.lat);
//      Serial.print(", ");
//      Serial.print(gps.longitude, 4); Serial.println(gps.lon);
//      Serial.print("Speed (knots): "); Serial.println(gps.speed);
//      Serial.print("Angle: "); Serial.println(gps.angle);
//      Serial.print("Altitude: "); Serial.println(gps.altitude);
//      Serial.print("Satellites: "); Serial.println((int)gps.satellites);
        m_data.lat = gps.latitude;
        m_data.lon = gps.longitude;
        m_data.speed = gps.speed;
        m_data.satellites = gps.satellites;
        m_data.altitude = gps.altitude-zero_altitude;
        m_is_new_data = false;
        last_update = millis();
      }
    }
      



//        gps.crack_datetime(&m_timestamp.year, &m_timestamp.month, &m_timestamp.day, 
//                            &m_timestamp.hr, &m_timestamp.min, &m_timestamp.sec, 
//                            &m_timestamp.msec, &age);
//        
//        // Format date and time. 4 bytes => 4 bytes => xx HR MIN SEC
//        m_data.time = ((uint32_t)m_timestamp.hr << 16) | 
//                       ((uint32_t)m_timestamp.min << 8) | 
//                       (uint32_t)m_timestamp.sec;
//        // xx YEAR MONTH DAY
//        m_data.date = ((uint32_t)m_timestamp.year << 16) | 
//                       ((uint32_t)m_timestamp.month << 8) | 
//                       (uint32_t)m_timestamp.day;

      return true;
    }

    // Getters
//    TimeStamp timestamp() const { return m_timestamp; }
//    Coord coord() const { return m_coord; }
//    Stats stats() const {return m_stats; }

//    unsigned int satellites() const { return m_satellites; }   
//
//    double speed() const { return m_speed; }
//    double altitude() const { return m_altitude; }
//    double angle() const { return m_angle; }

private:
    constexpr static unsigned int GPS_BAUD_RATE = 9600;

    // Adafruit GPS used for parsing the GPS sentences
    Adafruit_GPS gps = Adafruit_GPS(&GPSSerial);
    // HardwareSerial port for interfacing with the GPS
    // NOTE: need to add software serial support as well
    HardwareSerial* port;

    // GPS timestamp, dd/mm/yy hr:min:sec:msec
    TimeStamp m_timestamp;
    // GPS coordinates
    Coord m_coord;
    // Transmission statistics such as recieved sentences and amount of failed parsing
    Stats m_stats;

    // Satellite count
    unsigned int m_satellites;
    
    // Speed in m/s, altitude in m, and angle in degrees
    double m_speed, m_altitude, m_angle;

    float zero_altitude = 0;
    float accumulated_altitude = 0;
    int sample_counter = 0;
    const int SAMPLES = 10;
    bool led_state = HIGH;
    bool m_is_new_data = false;

    bool check() {
     // Read data from GPS directly
      char c = gps.read();
        
      // If sentence is received, check the checksum and parse it if valid
      if (gps.newNMEAreceived()) {
        if (!gps.parse(gps.lastNMEA())) // Resets the newNMEAreceived() flag to false
          return false; // If we fail to parse, return false indicating that a new message was received but not parsed
      } else {
        return false; // No new message received;
      }
      
      return true;
    }
    
};
