/*
 * implementation of a system that is used for a full system test
 * all sensors are live and full communication with the ground station
 */

 #pragma once

#include "Arduino.h"
#include "System.h"
#include "IMU_MPU9250.h"


class Test_System : public System {
  public:
    Test_System();
    ~Test_System();
    void initSystem() override;
    void updateSystem() override;
  
  protected:

  
  private:
    String type = String("This is a test system");
    TEST_IMU_MPU9250 *imu;
  
};
