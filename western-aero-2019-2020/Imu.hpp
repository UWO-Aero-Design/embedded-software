/** \file Imu.hpp
   @brief All code relating to the IMU sensor
*/

#pragma once

// Include for sensor interfaces; imu needed
#include "src/aero-cpp-lib/include/Sensors.hpp"
#include "Wire.h"
#include "src/MPU9250/MPU9250.h"
#include "src/MPU9250/quaternionFilters.h"

/**
   @brief Class for a IMU sensor made by SparkFun
   @details It is an I2C sensor with an accelerometer, magnetometer and gyroscope
*/
class ImuMpu9250 : public aero::sensor::IMU {
  public:

    /**
       @brief Construct a new IMU sensor object
    */
    ImuMpu9250() {};

    /**
       @brief Initialize sensor object

       @return true If init succeeded
       @return false If init failed
    */
    bool init() override {
      pinMode(m_int_pin, INPUT);
      digitalWrite(m_int_pin, LOW);

      // Read the WHO_AM_I register, this is a good test of communication
      byte c = m_imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

      if (c == 0x71) // WHO_AM_I should always be 0x71
      {
        // Serial.println(F("MPU9250 is online..."));


        // Calibrate gyro and accelerometers, load biases in bias registers
        m_imu.calibrateMPU9250(m_imu.gyroBias, m_imu.accelBias);

        m_imu.initMPU9250();
        // Initialize device for active mode read of acclerometer, gyroscope, and
        // temperature


        // Read the WHO_AM_I register of the magnetometer, this is a good test of
        // communication
        byte d = m_imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
        if (d != 0x48)
        {
          // Communication failed, stop here
          m_initialized = false;
          Serial.println(F("unable to communicate with AK8963"));
          return m_initialized;

        }

        else {
          m_initialized = true;
          // Get magnetometer calibration from AK8963 ROM
          m_imu.initAK8963(m_imu.factoryMagCalibration);
          // Initialize device for active mode read of magnetometer




          // Get sensor resolutions, only need to do this once
          m_imu.getAres();
          m_imu.getGres();
          m_imu.getMres();

          // The next call delays for 4 seconds, and then records about 15 seconds of
          // data to calculate bias and scale.
          // m_imu.magCalMPU9250(m_imu.magBias, m_imu.magScale);
        }
      } // if (c == 0x71)
      else {
        m_initialized = false;
        return m_initialized;

        // Communication failed, stop here

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

      // If intPin goes high, all data registers have new data
      // On interrupt, check if data ready interrupt
      if (m_imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        get_data();
      }

      compute_ypr();


      m_data.ax    = (int16_t)(m_imu.ax    * 100);
      m_data.ay    = (int16_t)(m_imu.ay    * 100);
      m_data.az    = (int16_t)(m_imu.az    * 100);
      m_data.gx    = (int16_t)(m_imu.gx    * 100);
      m_data.gx    = (int16_t)(m_imu.gy    * 100);
      m_data.gx    = (int16_t)(m_imu.gz    * 100);
      m_data.mx    = (int16_t)(m_imu.mx    * 100);
      m_data.mx    = (int16_t)(m_imu.my    * 100);
      m_data.mx    = (int16_t)(m_imu.mz    * 100);
      m_data.yaw   = (int16_t)((m_imu.yaw - yaw_zero)   * 100);
      m_data.pitch = (int16_t)((m_imu.pitch - pitch_zero) * 100);
      m_data.roll  = (int16_t)((m_imu.roll - roll_zero)  * 100);

      m_imu.count = millis();
      m_imu.sumCount = 0;
      m_imu.sum = 0;

      return true;
    }

    void calibrate() {
      static constexpr const int SAMPLES = 100;
      static constexpr const int PRE_SAMPLES = 1000;
      int sample_counter = 0;
      int yaw_sum = 0, pitch_sum = 0, roll_sum = 0;
      while(sample_counter < SAMPLES + PRE_SAMPLES) {
        if (m_imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
          get_data();
          if(PRE_SAMPLES - sample_counter <= 0) {
            yaw_sum += m_imu.yaw;
            pitch_sum += m_imu.pitch;
            roll_sum += m_imu.roll;
          }
          sample_counter++;
        }
        compute_ypr();
      }
      yaw_zero = yaw_sum / SAMPLES;
      pitch_zero = pitch_sum / SAMPLES;
      roll_zero = roll_sum / SAMPLES;
    }


  private:
    // Track whether sensor has been initialized
    bool m_initialized = false;
    bool m_mpu_initialized = false;
    bool m_mag_initialized = false;

    const int m_int_pin = 28;

    float yaw_zero, pitch_zero, roll_zero;

    int MPU9250_ADDRESS = MPU9250_ADDRESS_AD0;
    int I2Cclock = 400000;

    MPU9250 m_imu = MPU9250(MPU9250_ADDRESS, Wire, I2Cclock);


    void get_data() {
      m_imu.readAccelData(m_imu.accelCount);  // Read the x/y/z adc values

      // Now we'll calculate the accleration value into actual g's
      // This depends on scale being set
      m_imu.ax = (float)m_imu.accelCount[0] * m_imu.aRes; // - m_imu.accelBias[0];
      m_imu.ay = (float)m_imu.accelCount[1] * m_imu.aRes; // - m_imu.accelBias[1];
      m_imu.az = (float)m_imu.accelCount[2] * m_imu.aRes; // - m_imu.accelBias[2];

      m_imu.readGyroData(m_imu.gyroCount);  // Read the x/y/z adc values

      // Calculate the gyro value into actual degrees per second
      // This depends on scale being set
      m_imu.gx = (float)m_imu.gyroCount[0] * m_imu.gRes;
      m_imu.gy = (float)m_imu.gyroCount[1] * m_imu.gRes;
      m_imu.gz = (float)m_imu.gyroCount[2] * m_imu.gRes;

      m_imu.readMagData(m_imu.magCount);  // Read the x/y/z adc values

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental
      // corrections
      // Get actual magnetometer value, this depends on scale being set
      m_imu.mx = (float)m_imu.magCount[0] * m_imu.mRes
                 * m_imu.factoryMagCalibration[0] - m_imu.magBias[0];
      m_imu.my = (float)m_imu.magCount[1] * m_imu.mRes
                 * m_imu.factoryMagCalibration[1] - m_imu.magBias[1];
      m_imu.mz = (float)m_imu.magCount[2] * m_imu.mRes
                 * m_imu.factoryMagCalibration[2] - m_imu.magBias[2];

    }

    void compute_ypr() {
      // Must be called before updating quaternions!
      m_imu.updateTime();
      MahonyQuaternionUpdate(m_imu.ax, m_imu.ay, m_imu.az, m_imu.gx * DEG_TO_RAD,
                             m_imu.gy * DEG_TO_RAD, m_imu.gz * DEG_TO_RAD, m_imu.my,
                             m_imu.mx, m_imu.mz, m_imu.deltat);






      // Serial print and/or display at 0.5 s rate independent of data rates
      m_imu.delt_t = millis() - m_imu.count;



      m_imu.yaw   = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ()
                                  * *(getQ() + 3)), *getQ() * *getQ() + * (getQ() + 1)
                          * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) - * (getQ() + 3)
                          * *(getQ() + 3));
      m_imu.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ()
                                  * *(getQ() + 2)));
      m_imu.roll  = atan2(2.0f * (*getQ() * *(getQ() + 1) + * (getQ() + 2)
                                  * *(getQ() + 3)), *getQ() * *getQ() - * (getQ() + 1)
                          * *(getQ() + 1) - * (getQ() + 2) * *(getQ() + 2) + * (getQ() + 3)
                          * *(getQ() + 3));
      m_imu.pitch *= RAD_TO_DEG;
      m_imu.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination

      m_imu.yaw  -= 6.31;
      m_imu.roll *= RAD_TO_DEG;
    }
};
