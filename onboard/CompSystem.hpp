/** \file CompSystem.hpp
   @brief Fully-fledged system that will be used during competiton
*/

#pragma once

#include "System.hpp"
#include "src/Message/pb_encode.h"
#include "src/Message/pb_decode.h"
#include "src/Message/message.pb.h"
#include <stdio.h>

#define TELEMETRY_SEND_TIMEOUT 100


/***************************************************************************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/******************** FULL SYSTEM USED FOR COMPETITION  ********************/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/***************************************************************************/
/*!
  @brief Implementation of a competition system
*/
class CompSystem : public System {

  public:
    // Description of the system for printing
    static constexpr const char* DESCRIPTION = "Competition System";

    CompSystem() {
    }

    ~CompSystem() {
    }

    // Init method starts serial and builds test data
    bool init() override {
      Wire.begin();
      bool is_success = true;

      // Serial object initialization
      if (imu.init()) {
        Serial.println("IMU online.");
      }
      else {
        Serial.println("Error connecting to IMU.");
        is_success = false;
      }
      if (enviro.init()) {
        Serial.println("Environment sensor online.");
      }
      else {
        Serial.println("Error connecting to environment sensor.");
        is_success = false;
      }
      if (radio.init()) {
        Serial.println("Radio online.");
      }
      else {
        Serial.println("Error connecting to radio.");
        is_success = false;
      }
      if (servos.init()) {
        Serial.println("Servo controller online.");
      }
      else {
        Serial.println("Error connecting to servo controller.");
        is_success = false;
      }
      if (gps.init()) {
        Serial.println("GPS online.");
      }
      else {
        Serial.println("Error connecting to GPS.");
        is_success = false;
      }

      Serial.println("\n");
      
      Serial.print("Calibrating Enviro...");
      if(enviro.calibrate()) {
        Serial.println("Done.");
      }
      else {
        Serial.println("Error calibrating the enviro.");
        is_success = false;
      }

//      Serial.print("Calibrating IMU...");
//      imu.calibrate();
//      Serial.println("Done.");
return true;
    }

    bool update() override {
      
      // ---- tick sensor state --- //
      bool imu_success = imu.update();
      bool enviro_success = enviro.update();
      bool gps_success = gps.update();

      // ---- collect data from sensors --- //
      if(imu_success) imu_data = imu.data();
      if(enviro_success) enviro_data = enviro.data();
      if(gps_success) gps_data = gps.data();

      // ---- receive message if ready --- //
      if(radio.ready()) {
        if(receive_message(&message_to_receive)) {
          
          // ---- reply with telemetry --- //
          Serial.print("Received message (Packet: ");
          Serial.print(message_to_receive.packet_number);
          Serial.println("), replying with telemetry");
          message_to_send = Message_init_zero;
          if(imu_success) imu_data_to_msg(&message_to_send, &imu_data);
          if(enviro_success) enviro_data_to_msg(&message_to_send, &enviro_data);
          if(gps_success) gps_data_to_msg(&message_to_send, &gps_data);
          send_telemetry(&message_to_send, &radio_data, packet_number++);
          
        }
        
        // update radio data
        radio_data = radio.data();
      }

      // ---- print debug data --- //
      if(millis() - last_print >= 500) {
        // fill print buffer with formatted text
        sprintf(print_buffer, "IMU [accel]: %-7.2f %-7.2f %-7.2f\tEnviro [A/T/P]: %-7.2f %-7.2f %-7.2f\tSats: %-i",
              imu_data.ax, imu_data.ay, imu_data.az,
              enviro_data.altitude, enviro_data.temperature, enviro_data.pressure , gps_data.satellites);
        Serial.println(print_buffer);
        last_print = millis();
      }

      return true;
    }

  protected:

  private:
    long last_print = 0;
    long last_led_update = 0;
    long last_telemetry_send = 0;
    uint32_t packet_number = 0;
    uint8_t send_buffer[128];
    Message message_to_receive;
    Message message_to_send;
    pb_ostream_t send_stream;
    pb_istream_t receive_stream;
    IMU imu_msg;
    Enviro enviro_msg;
    GPS gps_msg;
  
    // holds the nicely formatted sensor data string for printing
    char print_buffer[256];

    // Structs for data
    aero::def::IMU_t imu_data;
    aero::def::Pitot_t pitot_data;
    aero::def::GPS_t gps_data;
    aero::def::Enviro_t enviro_data;
    aero::def::Battery_t batt_data;
    aero::def::Radio_t radio_data;
    aero::def::SystemConfig_t system_config_data;
    aero::def::Status_t status_state_data;
    aero::def::Servos_t servos_data;
    aero::def::AirData_t airdata_data;
    aero::def::Commands_t commands_data;
    aero::def::DropAlgo_t dropalgo_data;

    // Sensors
    ImuIcm20948 imu;
    PhidgetPitotTube pitot {aero::teensy35::P14_PWM};
    AdafruitBMP280EnviroSensor enviro;
    Radio_Rfm95w radio;
    AdafruitGPS gps {&Serial2};
    ServoController servos;
    
    bool gps_fix = false;
    uint8_t GPS_FIX_PIN = aero::teensy35::P16;
    DropAlgo algo = DropAlgo(28.084217, -81.965614);

    // bitmasks for Commands_t struct of ParsedMessage_t
    const uint8_t OPEN_DOORS_MASK      = 0b00000001;
    const uint8_t CLOSE_DOORS_MASK     = 0b00000010;
    const uint8_t GLIDER_DROP_MASK     = 0b00000100;
    const uint8_t WATER_DROP_MASK      = 0b00001000;
    const uint8_t HABITATS_DROP_MASK   = 0b00010000;

    bool receive_message(Message *message) {
      bool message_received = false;
      uint8_t receive_buffer[radio.RECEIVE_BUFFER_SIZE];
      uint8_t bytes_received = radio.RECEIVE_BUFFER_SIZE;
      if(radio.receive(receive_buffer, &bytes_received)) {
        message_received = true;
        // init message
        *message = Message_init_zero;

        // set callback function for each command in command list
        // pass reference to this class so that static callback has access to instance members
        message->commands.funcs.decode = static_command_decode_callback;
        message->commands.arg = this;

        // set up receive stream and decode
        receive_stream = pb_istream_from_buffer(receive_buffer, bytes_received);
        bool status = pb_decode(&receive_stream, Message_fields, message);
        
        if (!status) {
          Serial.print("Error decoding message: "); Serial.println(PB_GET_ERROR(&receive_stream));
        }
        else {
          // message successful decoded
        }
        
      }
      else {
        Serial.println("Error receiving message");
      }

      return message_received;
    }

    void load_header(Message *msg, Message_Location recipient, uint32_t packet_number, Message_Status status, uint16_t rssi) {
      msg->sender = Message_Location::Message_Location_PLANE;
      msg->recipient = recipient;
      msg->packet_number = packet_number;
      msg->time = millis();
      msg->status = status;
      msg->rssi = rssi;
    }

    static bool static_command_decode_callback(pb_istream_t *istream, const pb_field_t *field, void **arg) { 
      // arg passed in is reference to the calling class
      // cast so we can call member function after decoding
      CompSystem *self = (CompSystem*)(*arg);

      // decode
      Message_Command command;
      if (istream != NULL && field->tag == Message_commands_tag) {
        if (!pb_decode_varint32(istream, (uint32_t*)&command)) {
          Serial.print("Error decoding command");
          return false;
        }

        // call member function
        self->handle_command(command);
        return true;
      }
      return false;
    }

    bool handle_command(Message_Command command) {
      switch(command) {
        case Message_Command_OPEN_DOORS:
          Serial.println("Message_Command_OPEN_DOORS");
          break;
        case Message_Command_CLOSE_DOORS:
          Serial.println("Message_Command_CLOSE_DOORS");
          break;
        case Message_Command_DROP_PAYLOADS:
          Serial.println("Message_Command_DROP_PAYLOADS");
          break;
        case Message_Command_DROP_GLIDERS:
          Serial.println("Message_Command_DROP_GLIDERS");
          drop_pada();
          break;
        default:
          Serial.println("Unknown command.");
          break;
      }
      return true;
    }

    bool temp = true;
    void drop_pada() {
      if(temp) {
        for(int i = 0; i < 16; i++) servos.open(i);
      }
      else {
        for(int i = 0; i < 16; i++) servos.close(i);
      }
      temp = !temp;
    }

    void send_telemetry(Message *message, aero::def::Radio_t *radio_data, uint32_t packet_number) {
      load_header(message, Message_Location::Message_Location_GROUND_STATION, packet_number, Message_Status::Message_Status_READY, radio_data->rssi);
      message_to_send.flight_stabilization = Message_FlightStabilization::Message_FlightStabilization_NONE;
      
      send_stream = pb_ostream_from_buffer(send_buffer, sizeof(send_buffer));
      bool status = pb_encode(&send_stream, Message_fields, message);

      if(status) {
        status = radio.send(send_buffer, send_stream.bytes_written);
        if(!status) {
          Serial.println("Error sending");
        }
      }
      else {
        Serial.println("Error encoding");
      }
    }

    void imu_data_to_msg(Message *message, aero::def::IMU_t *imu_data) {
      message->imu = IMU_init_zero;
      message->imu.ax = imu_data->ax;
      message->imu.ay = imu_data->ay;
      message->imu.az = imu_data->az;
      message->imu.gx = imu_data->gx;
      message->imu.gy = imu_data->gy;
      message->imu.gz = imu_data->gz;
      message->imu.mx = imu_data->mx;
      message->imu.my = imu_data->my;
      message->imu.mz = imu_data->mz;
      message->has_imu = true;
    }

    void enviro_data_to_msg(Message *message, aero::def::Enviro_t *enviro_data) {
      message->enviro = Enviro_init_zero;
      message->enviro.altitude = enviro_data->altitude;
      message->enviro.pressure = enviro_data->pressure;
      message->enviro.temperature = enviro_data->temperature;
      message->has_enviro = true;
    }

   void gps_data_to_msg(Message *message, aero::def::GPS_t *gps_data) {
      message->gps = GPS_init_zero;
      message->gps.fix = gps_data->fix;
      if(gps_data->fix) {
        message->gps.lat = gps_data->lat;
        message->gps.lon = gps_data->lon;
        message->gps.speed = gps_data->satellites;
        message->gps.altitude = gps_data->altitude;
        message->gps.time = gps_data->time;
        message->gps.date = gps_data->date;
        message->gps.HDOP = gps_data->HDOP;
        message->gps.quality = gps_data->quality;
      }
      message->has_gps = true;
    } 
};
