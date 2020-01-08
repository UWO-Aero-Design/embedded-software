#ifndef _GPS_H_
#define _GPS_H_

// Sensor interface
#include "src/aero-cpp-lib/include/Sensors.hpp"
#include "src/aero-cpp-lib/include/Utility.hpp"

// Adafruit library
#include "src/Adafruit_GPS/Adafruit_GPS.h"

#include <SoftwareSerial.h>

class AdafruitGPS : public aero::sensor::GPS {
public:

    // GMT Time Stamp
    struct TimeStamp {
        uint8_t hr;                                     
        uint8_t min;                                   
        uint8_t sec;                                  
        uint16_t msec;                            
        uint8_t year;                                     
        uint8_t month;                                    
        uint8_t day;                                      
    };

    struct Coord {
        double lat;
        double lon;
    };

    struct Connection {
        bool fix;
        uint8_t quality; // (0, 1, 2 = Invalid, GPS, DGPS)
        uint8_t satellites;
    };

    AdafruitGPS(HardwareSerial& port) : m_gps(&port){
        // m_port = port;

        // m_gps = Adafruit_GPS(port);
    }

    #if (defined(__AVR__) || defined(ESP8266)) && defined(USE_SW_SERIAL)
      AdafruitGPS(SoftwareSerial& port) : m_gps(&port){
          // m_port = port;
  
          // m_gps = Adafruit_GPS(port);
      }
    #endif

    bool init() override {
        
        // Begin GPS connection; check if hardware init succeeded
        bool success = m_gps.begin(BAUD_RATE);

        if(!success) {
            return false;
        }

        // RMC (Recommended minimum) and GGA (fix data)
        m_gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
        // Update rate, keep low for ebtter parsing
        m_gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
        // Antenna status
        m_gps.sendCommand(PGCMD_ANTENNA); 
        m_gps.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
        delay(1000);

        return true;
    }

    // Fails if no update occured
    bool update() override {
        if (m_gps.newNMEAreceived()) {
            if (!m_gps.parse(m_gps.lastNMEA())) {
                // Failed to parse a new message
                return false;
            }
            // Failed to receive a new message (in time)
            return false;
        }

        // Grab data from sensor
        m_timestamp.hr = m_gps.hour;
        m_timestamp.min = m_gps.minute;
        m_timestamp.sec = m_gps.seconds;
        m_timestamp.msec = m_gps.milliseconds;
        m_timestamp.year = m_gps.year;
        m_timestamp.month = m_gps.month;
        m_timestamp.day = m_gps.day;

        m_coord.lat = m_gps.latitudeDegrees;
        m_coord.lon = m_gps.longitudeDegrees;
        m_speed = m_gps.speed / 1.944;  // In knots; convert to m/s
        m_altitude = m_gps.altitude;
        m_angle = m_gps.angle;

        m_connection.fix = (bool) m_gps.fix;
        m_connection.quality = m_gps.fixquality;
        m_connection.satellites = m_gps.satellites;

        // Prepare data for message protocol
        m_data.lat = m_coord.lat / 100000;
        m_data.lon = m_coord.lon / 100000;
        m_data.speed = (uint32_t)(m_speed * 100); // max 2 decimal precision, is around ~ 650 m/s
        m_data.altitude = (uint32_t)(m_speed * 10); // max is 1 decimal precision, ~6500 m

        m_data.satellites = m_connection.satellites;
        // Set MSB of satellites if fix
        if(m_connection.fix) {
            m_data.satellites = aero::bit::set(m_data.satellites, 7);
        } else {
            m_data.satellites = aero::bit::clear(m_data.satellites, 7);
        }

        m_data.time = ((uint32_t)m_timestamp.hr << 16) | 
                        ((uint32_t)m_timestamp.min << 8) | 
                        (uint32_t)m_timestamp.sec;

        m_data.date = ((uint32_t)m_timestamp.year << 16) | 
                        ((uint32_t)m_timestamp.month << 8) | 
                        (uint32_t)m_timestamp.day;

        return true;
    }

    
    TimeStamp timestamp() const {
        return m_timestamp;
    }

    Coord coord() const {
        return m_coord;
    }

    Connection connection() const {
        return m_connection;
    }   

    double speed() const {
        return m_speed;
    }

    double altitude() const {
        return m_altitude;
    }

    double angle() const {
        return m_angle;
    }

protected:
private:
    Adafruit_GPS m_gps;
    Stream* m_port;

    TimeStamp m_timestamp;
    Coord m_coord;
    Connection m_connection;
    double m_speed, m_altitude, m_angle;
    
    constexpr static unsigned int BAUD_RATE = 9600;
};

#endif
