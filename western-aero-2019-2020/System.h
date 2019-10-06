/*
 * abstract class for defining how a system should be structured
 */

#pragma once

#include "Arduino.h"
#include "ImuMpu9250.h"


// abstract class for defining how a system should be structured
class System {
  public:
    System(){}
    virtual ~System(){}
    virtual void init() = 0;
    virtual void update() = 0;  
};

// implementation of the competiton system
class CompSystem : public System {
  public:
    CompSystem();
    ~CompSystem();
    void init() override;
    void update() override;

  private:
    const String type = String("This is a competition system");
    ImuMpu9250 *imu;
  
};

// implementation of the test system
class TestSystem : public System {
  public:
    TestSystem();
    ~TestSystem();
    void init() override;
    void update() override;

  private:
    String type = String("This is a test system");
    TestImuMpu9250 *imu;
  
};

// factory-styled pattern for creating a system
class SystemSelect {
  public:
    SystemSelect();
    ~SystemSelect();
    enum SystemType { CompSystem_t = 0b00001111, TestSystem_t = 0b00000000 };
    static System *system_select(uint8_t type) {
      switch(type) {
        case 0b00000000:
          return new TestSystem();
          break;
        case 0b00001111:
        default:
          return new CompSystem();
          break;
      }
    };
};
