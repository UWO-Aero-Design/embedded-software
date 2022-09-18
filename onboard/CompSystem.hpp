/** \file CompSystem.hpp
   @brief Fully-fledged system that will be used during competiton
*/

#pragma once

#define BUFFER_SIZE 128
#define TELEMTRY_PRINT_INTERVAL 500

#include "System.hpp"
#include "src/Message/pb_encode.h"
#include "src/Message/pb_decode.h"
#include "src/Message/telemetry.pb.h"
#include "src/Message/command.pb.h"


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

      leds.attach(&heart_beat_animation);
      leds.attach(&radio_animation);
      leds.attach(&error_animation);
      leds.attach(&gps_fix_animation);

      if(!is_success) {
        error_animation.ping();
      }

      buttons.on(Pins::BUTTON_1, TransitionType_t::RISING_EDGE, [this](int button_number, void *context) {
        set_pada_mechanism(ServoState::ServoState_OPEN);
      });

      buttons.on(Pins::BUTTON_2, TransitionType_t::RISING_EDGE, [this](int button_number, void *context) {
        set_pada_mechanism(ServoState::ServoState_CLOSE);
      });
      
      return is_success;
      
    }

    bool update() override {
      
      // ---- tick sensor state --- //
      bool imu_success = imu.update();
      bool enviro_success = enviro.update();
      bool gps_success = gps.update();
      bool radio_success = radio.update();
      leds.update();
      buttons.update();

      // ---- collect data from sensors --- //
      if(imu_success) imu_data = imu.data();
      if(enviro_success) enviro_data = enviro.data();
      if(gps_success) gps_data = gps.data();

      if(gps_success && gps_data.fix == 1) {
        gps_fix_animation.ping();
      }

      // ---- receive message if ready --- //
      if(radio.ready()) {
        Command message_to_receive;
        uint8_t message_bytes_received;
        uint32_t received_packet_number = 0;
        if(receive_message(&message_to_receive, &message_bytes_received, &received_packet_number)) {
          radio_animation.ping();
          
          // ---- reply with telemetry --- //
          if(PRINT_RECEIVE_DEBUG) {
            Serial.print("Received message (Packet: ");
            Serial.print(message_to_receive.header.packet_number);
            Serial.println("), replying with telemetry");
          }
          Telemetry message_to_send = Telemetry_init_zero;
          if(imu_success) imu_data_to_msg(&message_to_send, &imu_data);
          if(enviro_success) enviro_data_to_msg(&message_to_send, &enviro_data);
//          if(gps_success) gps_data_to_msg(&message_to_send, &gps_data);
          battery_data_to_msg(&message_to_send, &battery_data);
          radio_data_to_msg(&message_to_send, &radio_data);
          if(received_packet_number != 0) {
            if(PRINT_ACK_DEBUG) {
              Serial.print("Acking command #");
              Serial.println(received_packet_number);
            }
            message_to_send.response_to = received_packet_number;
          }
          send_telemetry(&message_to_send, packet_number++);
          
        }
        
        // update radio data
        radio_data = radio.data();
      }

      // ---- print debug data --- //
      if(millis() - last_print >= TELEMTRY_PRINT_INTERVAL) {
        // fill print buffer with formatted text
        char print_buffer[BUFFER_SIZE];
        sprintf(print_buffer, "IMU [accel]: %-7.2f %-7.2f %-7.2f\tEnviro [A/T/P]: %-7.2f %-7.2f %-7.2f\tFix (sats): %-i (%-i)",
              imu_data.ax, imu_data.ay, imu_data.az,
              enviro_data.altitude, enviro_data.temperature, enviro_data.pressure , gps_data.fix, gps_data.satellites);
        Serial.println(print_buffer);
        last_print = millis();
      }

      return true;
    }

  protected:

  private:
    long last_print = 0; /*! The last time a message was printed to the serial monitor */
    uint32_t packet_number = 0; /*! The last time a message was printed to the serial monitor */
    const bool PRINT_RECEIVE_DEBUG = false;
    const bool PRINT_ACK_DEBUG = false;

    // Structs for data
    aero::def::IMU_t imu_data;
    aero::def::Pitot_t pitot_data;
    aero::def::GPS_t gps_data;
    aero::def::Enviro_t enviro_data;
    aero::def::Battery_t battery_data;
    aero::def::Radio_t radio_data;

    // Sensors
    ImuIcm20948 imu;
    AdafruitBMP280EnviroSensor enviro;
    Radio_Rfm95w radio;
    AdafruitGPS gps {&Serial2};
    ServoController servos;

    // LEDs
    LedController leds;
    DoublePulseAnimation heart_beat_animation{Pins::WHITE_LED, 100, 100, 500};
    HeartBeatAnimation radio_animation{Pins::YELLOW_LED, 500, HIGH, LOW};
    HeartBeatAnimation error_animation{Pins::RED_LED, 1000, HIGH, LOW};
    HeartBeatAnimation gps_fix_animation{Pins::BLUE_LED, 1000, HIGH, LOW};

    // buttons
    ButtonController buttons;
    
    
    bool gps_fix = false;
    uint8_t GPS_FIX_PIN = aero::teensy35::P16;

    bool receive_message(Command *message, uint8_t *bytes_received, uint32_t *received_packet_number) {
      *received_packet_number = 0;
      bool message_received = false;
      uint8_t receive_buffer[radio.RECEIVE_BUFFER_SIZE];
      *bytes_received = radio.RECEIVE_BUFFER_SIZE;
      if(radio.receive(receive_buffer, bytes_received)) {
        // init message
        *message = Command_init_zero;
        message_received = true;

        // set callback function for each command in command list
        // pass reference to this class so that static callback has access to instance members
        message->actuate_group.funcs.decode = static_actuate_group_callback;
        message->actuate_group.arg = this;

        // set up receive stream and decode
        pb_istream_t receive_stream = pb_istream_from_buffer(receive_buffer, *bytes_received);
        bool status = pb_decode(&receive_stream, Command_fields, message);

        // if unsuccessful
        if (!status) {
          Serial.print("Error decoding message: ");
          Serial.println(PB_GET_ERROR(&receive_stream));
          error_animation.ping();
        }
        else {
          // message successful decoded
          *received_packet_number = message->header.packet_number;
          if(message->reset_processor == true) {
            Serial.println("RESET");
            handle_processor_reset();
          }
        }
      }
      else {
        Serial.println("Error receiving message");
        error_animation.ping();
      }

      return message_received;
    }

    /**
     * @brief Prepares a header with for a message
     *
     * @param msg The message buffer
     * @param recipient The intended recipient of the message
     * @param packet_number The packet number to be loaded
     * @param status The current status of the aircraft
     * @param rssi The last RSSI of the radio
     */
    void load_header(Telemetry *message, Location recipient, uint32_t packet_number, Status status) {
      message->header.sender = Location::Location_PLANE;
      message->header.receiver = recipient;
      message->header.packet_number = packet_number;
      message->header.time = millis();
      message->header.status = status;
      message->has_header = true;
    }

    /**
     * @brief Callback function  wrapper to execute a member 
     * @detailed Since the PB decode routine expects a free floating function and we want to pass
     * a member function (which has an extra "this" argument at the end), this wrapper function will
     * accept (vois **arg) the pointer to the instance with which you want to call a member function 
     * for. The PB setup for this looks like:
     * message.<repeated property>.funcs.decode = static_command_decode_callback;
     * message.<repeated property>.arg = this; // "this" refers to the instance to have the member function called for
     * Note: all passed in arguments are defined by NanoPB
     *
     * @param istream The Pb in stream
     * @param field The field that is passed in
     * @param arg The user defined arguments
     */
    static bool static_actuate_group_callback(pb_istream_t *istream, const pb_field_t *field, void **arg) { 
      // arg passed in is reference to the calling class and arguments
      // cast so we can call member function after decoding
      CompSystem *self = (CompSystem*)(*arg);

      ActuateGroup actuate_group;
      bool status = pb_decode(istream, ActuateGroup_fields, &actuate_group);

      if(status) {
        self->set_pada_mechanism(actuate_group.state);
        return true;
      }
      else {
        return false;
      }
    }
    

    bool handle_processor_reset() {
      SCB_AIRCR = 0x05FA0004; // reset on Teensy
      return true;
    }

    void set_pada_mechanism(ServoState state) {
      if(state == ServoState::ServoState_OPEN) {
        servos.actuate(CommandId::PADA);
      }
      else {
        servos.reset(CommandId::PADA);
      }
    }

    void send_telemetry(Telemetry *message, uint32_t packet_number) {
      uint8_t send_buffer[BUFFER_SIZE];
      load_header(message, Location::Location_GROUND_STATION, packet_number, Status::Status_READY);

      pb_ostream_t send_stream = pb_ostream_from_buffer(send_buffer, sizeof(send_buffer));
      bool status = pb_encode(&send_stream, Telemetry_fields, message);

      if(status) {
        status = radio.send(send_buffer, send_stream.bytes_written);
        if(!status) {
          Serial.println("Error sending");
          error_animation.ping();
        }
      }
      else {
        Serial.print("Error encoding: ");
        Serial.println(PB_GET_ERROR(&send_stream));
        error_animation.ping();
      }
    }

    void imu_data_to_msg(Telemetry *message, aero::def::IMU_t *imu_data) {
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

    void enviro_data_to_msg(Telemetry *message, aero::def::Enviro_t *enviro_data) {
      message->enviro.altitude = enviro_data->altitude;
      message->enviro.pressure = enviro_data->pressure;
      message->enviro.temperature = enviro_data->temperature;
      message->has_enviro = true;
    }

   void gps_data_to_msg(Telemetry *message, aero::def::GPS_t *gps_data) {
      message->gps.fix = gps_data->fix;
      if(gps_data->fix) {
        message->gps.lat = gps_data->lat;
        message->gps.lon = gps_data->lon;
        message->gps.satellites = gps_data->satellites;
        message->gps.speed = gps_data->speed;
        message->gps.altitude = gps_data->altitude;
        message->gps.time = gps_data->time;
        message->gps.date = gps_data->date;
        message->gps.hdop = gps_data->HDOP;
        message->gps.quality = gps_data->quality;
      }
      message->has_gps = true;
    }

    void radio_data_to_msg(Telemetry *message, aero::def::Radio_t *radio_data) {
      message->plane_radio.rssi = radio_data->rssi;
      message->plane_radio.frequency_error = radio_data->frequencyError;
      message->plane_radio.snr = radio_data->snr;
      message->has_plane_radio = true;
    }

    void battery_data_to_msg(Telemetry *message, aero::def::Battery_t *battery_data) {
      message->battery.voltage = battery_data->voltage;
      message->battery.current = battery_data->current;
      message->has_battery = true;
    }
};
