#include "System.h"
#include "Wire.h"

CompSystem::CompSystem() {
  imu = new ImuMpu9250();
  radio = new Rfm95w();
}

CompSystem::~CompSystem() {
  delete imu;
  delete radio;
}

 /**
  * @brief Initialize the Competition System
  */
void CompSystem::init() {
  Wire.begin();
  imu->init();
  radio->init();
}

 /**
  * @brief Update the Competition system
  */
void CompSystem::update() {
  RawMessage_t raw_msg = msg_handler.build(ID::Plane, ID::Gnd);

  // clean message
  char buf[9+raw_msg.length];
  remove_msg_padding(raw_msg, buf);
  
  radio->send(buf);
}

 /**
  * @brief Remove any unused bytes in the RawMessage
  * 
  * @param msg 
  */
void CompSystem::remove_msg_padding(RawMessage_t msg, char *new_buf) {
  char *buf = (char *) &msg;
  for(int i = 0, j = 0; i < sizeof(msg); ++i, ++j) {
    // Skip empty parts of buffer - after data but before length, crc and end byte (5 bytes before data, up to 256 bytes of data and 4 bytes after data)
    if(i == msg.length+4) i = sizeof(msg) - 5;
    new_buf[j] = buf[i];
  }
}

void CompSystem::build_message() {
  // to be replaced with msg_handler.add_device(device->data());
    msg_handler.add_pitot(pitot_data);
//  msg_handler.add_imu(imu_data);
//  msg_handler.add_gps(gps_data);
//  msg_handler.add_enviro(enviro_data);
//  msg_handler.add_battery(battery_data);
//  msg_handler.add_config(system_config_data);
//  msg_handler.add_status(status_data);
//  msg_handler.add_actuators(servos_data);
//  msg_handler.add_airdata(air_data);
//  msg_handler.add_cmds(commands_data);
//  msg_handler.add_drop(drop_algo_data);
  
}

// to be removed 
void CompSystem::temp_fill_data() {
      // test data
      pitot_data.differential_pressure = 1;
//  imu_data.ax = 2;
//  imu_data.ay = 3;
//  imu_data.az = 4;
//  imu_data.gx = 5;
//  imu_data.gy = 6;
//  imu_data.gz = 7;
//  imu_data.mx = 8;
//  imu_data.my = 9;
//  imu_data.mz = 10;
//  imu_data.yaw = 11;
//  imu_data.pitch = 12;
//  imu_data.roll = 13;
//  gps_data.lat = 14;
//  gps_data.lon = 15;
//  gps_data.speed = 16;
//  gps_data.satellites = 17;
//  gps_data.altitude = 18;
//  gps_data.time = 19;
//  gps_data.date = 20;
//  enviro_data.pressure = 21;
//  enviro_data.humidity = 22;
//  enviro_data.temperature = 23;
//  battery_data.voltage = 24;
//  battery_data.current = 25;
//  status_data.rssi = 26;
//  status_data.state = 27;
//  servos_data.servo0 = 28;
//  servos_data.servo1 = 29;
//  servos_data.servo2 = 30;
//  servos_data.servo3 = 31;
//  servos_data.servo4 = 32;
//  servos_data.servo5 = 33;
//  servos_data.servo6 = 34;
//  servos_data.servo7 = 35;
//  servos_data.servo8 = 36;
//  servos_data.servo9 = 37;
//  servos_data.servo10 = 38;
//  servos_data.servo11 = 39;
//  servos_data.servo12 = 40;
//  servos_data.servo13 = 41;
//  servos_data.servo14 = 42;
//  servos_data.servo15 = 43;
//  air_data.ias = 44;
//  air_data.eas = 45;
//  air_data.tas = 46;
//  air_data.agl = 47;
//  air_data.pressure_alt = 48;
//  air_data.msl = 49;
//  air_data.density_alt = 50;
//  air_data.approx_temp = 51;
//  air_data.density = 52;
//  commands_data.drop = 53;
//  commands_data.servos = 54;
//  commands_data.pitch = 55;
//  drop_algo_data.heading = 56;
//  drop_algo_data.distance = 57;
}
