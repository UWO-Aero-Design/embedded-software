/*
 * implementation of a system that is used for a full system test
 * all sensors are live and full communication with the ground station
 */

 #pragma once

#include "Arduino.h"
#include "Sensor.h"

class IMU_MPU9250 : public Sensor {
  public:
    IMU_MPU9250();
    ~IMU_MPU9250();
    void initSensor() override;
    void updateSensor() override;
  
  protected:

  
  private:
    String type = String("This is an IMU");
  
};
