#include "Test_System.h"

Test_System::Test_System() {
  
}

Test_System::~Test_System() {
  
}


void Test_System::initSystem() {
  imu = new TEST_IMU_MPU9250();
}

void Test_System::updateSystem() {
  Serial.println(type);
  Serial.println(imu->type);
}
