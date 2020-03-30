/** \file GPS.hpp
 * @brief All code relating to the GPS sensor
 */

#pragma once

// Sensor interface
#include "src/aero-cpp-lib/include/Sensors.hpp"
#include "src/aero-cpp-lib/include/Utility.hpp"

#include "Arduino.h"
#include "src/Adafruit_GPS/Adafruit_GPS.h"

/**
 * @brief AdafruitGPS adapter class 
 * @details Communication to the sensor is handled via serial and parsing using TinyGPS
 */
class AdafruitGPS : public aero::sensor::GPS {
public: 

    // Scalars for message protocol
    static constexpr unsigned int LAT_SCALAR = 100000;
    static constexpr unsigned int LON_SCALAR = 100000;
    static constexpr unsigned int ALT_SCALAR = 10;
    static constexpr unsigned int SPEED_SCALAR = 100;

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
      //gps = Adafruit_GPS(port);
    }

    /**
     * @brief Initialize the GPS module
     * 
     * @return true If update succeeded
     * @return false If update failed
     */
    bool init() override {
        // 9600 NMEA is the default baud rate this module but note that some use 4800
        gps.begin(9600);
        
        gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // The data we are requesting
        gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
      
        // Request updates on antenna status, comment out to keep quiet
        gps.sendCommand(PGCMD_ANTENNA);

        delay(1000);
      
        // Ask for firmware version
        Serial2.println(PMTK_Q_RELEASE);
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
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        if (!gps.parse(gps.lastNMEA())) // this also sets the newNMEAreceived() flag to false
          return; // we can fail to parse a sentence in which case we should just wait for another
      }
      if(millis() - last_update >= 1000) {
        if (gps.fix) {
          if(gps.satellites != 255 && first_fix_pulse == false) {
            first_fix_pulse = true;
            Serial.println("GPS has fix. Calibrating...");
          }
          if(sample_counter < SAMPLES) {
            accumulated_altitude += gps.altitude;
            sample_counter++;
            led_state = !led_state;
            digitalWrite(20, led_state);
          }
          if(sample_counter == SAMPLES) {
            zero_altitude = accumulated_altitude / SAMPLES;
            Serial.println("Done GPS calibration.");
            sample_counter++;
            led_state = LOW;
            digitalWrite(20, LOW);
          }
          Serial.print(gps.latitude, 8); Serial.print("\t"); Serial.println(gps.longitude, 8);
          m_data.satellites = gps.satellites;
          m_data.lat = (int32_t)(gps.latitude * LAT_SCALAR);
          m_data.lon = (int32_t)(gps.longitude * LON_SCALAR);
          m_data.altitude = (uint16_t)((gps.altitude-zero_altitude)* ALT_SCALAR);
          m_data.speed = (uint16_t)(gps.speed/3.6 * SPEED_SCALAR); // need to convert to m/s
          m_angle = gps.angle;
          last_update = millis();
        }
        else {
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
    TimeStamp timestamp() const { return m_timestamp; }
    Coord coord() const { return m_coord; }
    Stats stats() const {return m_stats; }

    unsigned int satellites() const { return m_satellites; }   

    double speed() const { return m_speed; }
    double altitude() const { return m_altitude; }
    double angle() const { return m_angle; }

private:
    bool first_fix_pulse = false;
    constexpr static unsigned int GPS_BAUD_RATE = 9600;

    // Adafruit GPS used for parsing the GPS sentences
    Adafruit_GPS gps = Adafruit_GPS(&Serial3);
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
