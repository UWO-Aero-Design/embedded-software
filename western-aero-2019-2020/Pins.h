
#pragma once

typedef enum {

    // dip switches
    DIPSWITCH_1 = 24,
    DIPSWITCH_2 = 25,
    DIPSWITCH_3 = 26,
    DIPSWITCH_4 = 27,

    // buttons
    BUTTON_1 = 14,
    BUTTON_2 = 15,
    
    // i2c
    SDA_0 = 18,
    SCL_0 = 19,

    // imu
    IMU_INT = 22,

    // battery
    BAT_LEVEL = 26,

    // daughter board
    AUX = 27,
    SELECT = 31,

    // leds
    RED_LED = 38,
    YELLOW_LED = 39,
    WHITE_LED = 40,
    ORANGE_LED = 41,
    
    // radio
    RADIO_INT = 30,
    RADIO_EN = 32,
    RADIO_RST = 9,

    // gps
    GPS_FIX = 3,
    GPS_EN = 4,
    GPS_PPS = 5

} Pins;
