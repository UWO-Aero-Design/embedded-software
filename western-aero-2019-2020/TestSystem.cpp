#include "System.hpp"

TestSystem::TestSystem() {
  
}

TestSystem::~TestSystem() {
  
}


void TestSystem::init() {
  imu = new TestImuMpu9250();
}

void TestSystem::update() {
  Serial.println(type);
  Serial.println(imu->type);
}
