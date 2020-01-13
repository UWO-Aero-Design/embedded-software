#ifndef _GPS_H_
#define _GPS_H_

// Sensor interface
#include "src/aero-cpp-lib/include/Sensors.hpp"
#include "src/aero-cpp-lib/include/Utility.hpp"

// Adafruit library
#include "src/Adafruit_GPS/Adafruit_GPS.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

#include <TinyGPS.h>

class AdafruitGPS : public aero::sensor::GPS {
public:

    // GMT Time Stamp
    struct TimeStamp {
        byte hr;                                     
        byte min;                                   
        byte sec;                                  
        byte msec;                            
        int year;                                     
        byte month;                                    
        byte day;                                      
    };

    struct Stats {
      unsigned long chars;
      unsigned short seqs;
      unsigned short fails;
    };

    struct Coord {
        double lat;
        double lon;
    };

    AdafruitGPS(HardwareSerial *port) {
      this->port = port;
    }

    bool init() override {
        // Begin GPS connection; check if hardware init succeeded
        Serial.begin(115200);
        delay(1000);

        port->begin(9600);
        
        return true;
    }

    // Fails if no update occured
    bool update() override {
        float flat, flon;
        unsigned long age, chars = 0;
        unsigned short sentences = 0, failed = 0;

        // Get position data and statistics
        gps.f_get_position(&flat, &flon, &age);
        gps.stats(&chars, &sentences, &failed);

        // Satellites
        m_satellites = gps.satellites();
        m_data.satellites = m_satellites;
        
        // Latitude
        m_coord.lat = flat;
        m_data.lat = (int32_t)(m_coord.lat * 10000000);

        // Longitude
        m_coord.lon = flon;
        m_data.lon = (int32_t)(m_coord.lon * 10000000);

        // Altitude
        m_altitude = gps.f_altitude();
        m_data.altitude = (uint16_t)(m_altitude * 10); // max is 1 decimal precision, ~6500 m

        // Speed; in m/s
        m_speed = gps.f_speed_kmph()/3.6;
        m_data.speed = (uint16_t)(m_speed * 100); // max 2 decimal precision, is around ~ 650 m/s

        // Course; not a part of data
        m_angle = gps.f_course();

        gps.crack_datetime(&m_timestamp.year, 
          &m_timestamp.month, 
          &m_timestamp.day, 
          &m_timestamp.hr, 
          &m_timestamp.min, 
          &m_timestamp.sec, 
          &m_timestamp.msec, 
          &age);
        
        m_data.time = ((uint32_t)m_timestamp.hr << 16) | 
                       ((uint32_t)m_timestamp.min << 8) | 
                       (uint32_t)m_timestamp.sec;

        m_data.date = ((uint32_t)m_timestamp.year << 16) | 
                       ((uint32_t)m_timestamp.month << 8) | 
                       (uint32_t)m_timestamp.day;

        //smartdelay(1000);

        return true;
    }

    void delay(unsigned long ms)
    {
      unsigned long start = millis();
      do {
        while (port->available())
          gps.encode(port->read());
      } while (millis() - start < ms);
    }

    
    TimeStamp timestamp() const {
        return m_timestamp;
    }

    Coord coord() const {
        return m_coord;
    }

    unsigned int satellites() const {
        return m_satellites;
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

    Stats stats() const {
      return m_stats;
    }

protected:
private:
    TinyGPS gps;
    HardwareSerial* port;
    TimeStamp m_timestamp;
    Coord m_coord;
    unsigned int m_satellites;
    Stats m_stats;
    double m_speed, m_altitude, m_angle;
    
    constexpr static unsigned int BAUD_RATE = 9600;
};

#endif
