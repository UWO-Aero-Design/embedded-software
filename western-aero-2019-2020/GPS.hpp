#ifndef _GPS_H_
#define _GPS_H_

// Sensor interface
#include "src/aero-cpp-lib/include/Sensors.hpp"
#include "src/aero-cpp-lib/include/Utility.hpp"

#include "Arduino.h"
#include "src/TinyGPS/TinyGPS.h"

/**
 * @brief AdafruitGPS adapter class 
 * @details Communication to the sensor is handled via serial and parsing using TinyGPS
 */
class AdafruitGPS : public aero::sensor::GPS {
public: 

    // Scalars for message protocol
    static constexpr unsigned int LAT_SCALAR = 10000000;
    static constexpr unsigned int LON_SCALAR = 10000000;
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
    }

    /**
     * @brief Initialize the GPS module
     * 
     * @return true If update succeeded
     * @return false If update failed
     */
    bool init() override {
        // Begin GPS connection; check if hardware init succeeded
        port->begin(GPS_BAUD_RATE);
        while(!port){ /* wait for port to begin */ }

        return true;
    }

    /**
     * @brief Grabs data from the GPS module and updates internal variables
     * 
     * @return true If update succeeded
     * @return false If update failed
     */
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
        m_data.lat = (int32_t)(m_coord.lat * LAT_SCALAR);

        // Longitude
        m_coord.lon = flon;
        m_data.lon = (int32_t)(m_coord.lon * LON_SCALAR);

        // Altitude
        m_altitude = gps.f_altitude();
        m_data.altitude = (uint16_t)(m_altitude * ALT_SCALAR); // max is 1 decimal precision, ~6500 m

        // Speed; in m/s
        m_speed = gps.f_speed_kmph()/3.6;
        m_data.speed = (uint16_t)(m_speed * SPEED_SCALAR); // max 2 decimal precision, is around ~ 650 m/s
        
        // Course; not a part of data
        m_angle = gps.f_course();

        gps.crack_datetime(&m_timestamp.year, &m_timestamp.month, &m_timestamp.day, 
                            &m_timestamp.hr, &m_timestamp.min, &m_timestamp.sec, 
                            &m_timestamp.msec, &age);
        
        // Format date and time. 4 bytes => 4 bytes => xx HR MIN SEC
        m_data.time = ((uint32_t)m_timestamp.hr << 16) | 
                       ((uint32_t)m_timestamp.min << 8) | 
                       (uint32_t)m_timestamp.sec;
        // xx YEAR MONTH DAY
        m_data.date = ((uint32_t)m_timestamp.year << 16) | 
                       ((uint32_t)m_timestamp.month << 8) | 
                       (uint32_t)m_timestamp.day;

        return true;
    }

    /**
     * @brief This delay is used to pause execution while also allowing the GPS buffer to fill up
     * @param ms Time for the delay
     * @note It is CRUCIAL that when using this gps class, you use this delay and not the regular arduino delay
     */
    void delay(unsigned long ms)
    {
        unsigned long start = millis();
        do {
            while (port->available())
            gps.encode(port->read());
        } while (millis() - start < ms);
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
    constexpr static unsigned int GPS_BAUD_RATE = 9600;

    // Tiny GPS used for parsing the GPS sentences
    TinyGPS gps;
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
    
};

#endif
