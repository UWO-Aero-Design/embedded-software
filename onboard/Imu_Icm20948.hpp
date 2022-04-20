/** \file Imu.hpp
   @brief All code relating to the IMU sensor
*/

#pragma once

// Include for sensor interfaces; imu needed
#include "src/aero-cpp-lib/include/Sensors.hpp"
#include "Wire.h"
#include "src/ICM20948/ICM_20948.h"

/**
   @brief Class for a IMU sensor made by SparkFun
   @details It is an I2C sensor with an accelerometer, magnetometer and gyroscope
*/
class ImuIcm20948 : public aero::sensor::IMU {
  public:

    /**
       @brief Construct a new IMU sensor object
    */
    ImuIcm20948() {};

    /**
       @brief Initialize sensor object

       @return true If init succeeded
       @return false If init failed
    */
    bool init() override {
      m_imu.begin(Wire, 0);
      if(m_imu.status != ICM_20948_Stat_Ok) {
        m_initialized = false;
      }
      else {
        m_initialized = true;
      }

      return m_initialized;
    }

    /**
       @brief Update sensor value by reading registers over I2C

       @return true sensor value updated
       @return false sensor value not updated
    */
    bool update() override {
      // If system is not initialized, return error
      if (!m_initialized) {
        return false;
      }

      if(m_imu.dataReady()) {
        m_imu.getAGMT();
        m_data.ax = m_imu.agmt.acc.axes.x;
        m_data.ay = m_imu.agmt.acc.axes.y;
        m_data.az = m_imu.agmt.acc.axes.z;
        m_data.gx = m_imu.agmt.gyr.axes.x;
        m_data.gy = m_imu.agmt.gyr.axes.y;
        m_data.gz = m_imu.agmt.gyr.axes.z;
        m_data.mx = m_imu.agmt.mag.axes.x;
        m_data.my = m_imu.agmt.mag.axes.y;
        m_data.mz = m_imu.agmt.mag.axes.z;
      }
      

      return true;
    }

    void calibrate() {
//      static constexpr const int SAMPLES = 100;
//      static constexpr const int PRE_SAMPLES = 1000;
//      int sample_counter = 0;
//      int yaw_sum = 0, pitch_sum = 0, roll_sum = 0;
//      while(sample_counter < SAMPLES + PRE_SAMPLES) {
//        if (m_imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
//          get_data();
//          if(PRE_SAMPLES - sample_counter <= 0) {
//            yaw_sum += m_imu.yaw;
//            pitch_sum += m_imu.pitch;
//            roll_sum += m_imu.roll;
//          }
//          sample_counter++;
//        }
//        compute_ypr();
//      }
//      yaw_zero = yaw_sum / SAMPLES;
//      pitch_zero = pitch_sum / SAMPLES;
//      roll_zero = roll_sum / SAMPLES;
    }


  private:
    // Track whether sensor has been initialized
    bool m_initialized = false;

    float yaw_zero, pitch_zero, roll_zero;

    ICM_20948_I2C m_imu;


};
