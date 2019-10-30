#include "TestMessages.h"
#include "System_Select.h"

#define led 13

using namespace aero;
using namespace aero::def;

const bool SEND_IMU = true;
const bool SEND_PITOT = true;
const bool SEND_GPS = true;
const bool SEND_ENV = true;
const bool SEND_BATT = true;
const bool SEND_SYSTEM = true;
const bool SEND_STATUS = true;
const bool SEND_SERVOS = true;
const bool SEND_AIRDATA = true;
const bool SEND_CMDS = true;
const bool SEND_DROPALGO = true;

Message msg_handler;

IMU_t imu = random_imu();
Pitot_t pitot = random_pitot();
GPS_t gps = test_gps();
Enviro_t enviro = test_enviro();
Battery_t batt = test_battery();
SystemConfig_t system_config = test_system();
Status_t status_state = test_status();
Servos_t servos = test_servos();
AirData_t airdata = test_airdata();
Commands_t commands = test_commands();
DropAlgo_t dropalgo = test_dropalgo();

void setup() {
  // Init serial port
  Serial.begin(115200);

  // Need to seed random number generator from setup() if using random
  randomSeed(analogRead(0));

  pinMode(led, OUTPUT);
  Serial.println("Hello World!");
  
}

void loop() {
  // Add to message buffer
  if(SEND_IMU) 
    msg_handler.add_imu(imu);
  if(SEND_PITOT)
    msg_handler.add_pitot(pitot);
  if(SEND_GPS)
    msg_handler.add_gps(gps);
  if(SEND_ENV)
    msg_handler.add_enviro(enviro);
  if(SEND_BATT)
    msg_handler.add_battery(batt);
  if(SEND_SYSTEM)
    msg_handler.add_config(system_config);
  if(SEND_STATUS)
    msg_handler.add_status(status_state);
  if(SEND_SERVOS)
    msg_handler.add_actuators(servos);
  if(SEND_AIRDATA)
    msg_handler.add_airdata(airdata);
  if(SEND_CMDS)
    msg_handler.add_cmds(commands);
  if(SEND_DROPALGO)
    msg_handler.add_drop(dropalgo);
  
  RawMessage_t raw_msg = msg_handler.build(ID::G1, ID::G2);
  char *buf = (char *) &raw_msg;

  // Send message. Make sure to skip the part of the buffer that is empty
  for(int i = 0; i < sizeof(raw_msg); ++i) {
    // Skip empty parts of buffer
    if(i == raw_msg.length+6) {
      i = 256+6;
    }

    Serial.print((char)buf[i], HEX);
    Serial.print(' ');
  }
  
  Serial.print("\n");

  // Message every second
  delay(1000);

  ParsedMessage_t parsed = msg_handler.parse((uint8_t *)buf);
  IMU_t *imuu = ( reinterpret_cast<IMU_t*>( parsed.segments[ static_cast<int>(Signature::IMU) ] ) );
  //Serial.print("Hello: ");
  //Serial.print( imuu->ax );
  //Serial.print( ' ' );
  //Serial.println( imuu->gy );
}
