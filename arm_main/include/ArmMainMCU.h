/**
 * @file ArmMainMCU.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Arm (Socket) Main MCU pins
 *
 */
#pragma once


#if !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)

//------------------------------------------------------------------------------------------------//
//   DOIT ESP32 Devkit V1 (URC 2025, Socket V2)
//------------------------------------------------------------------------------------------------//

#    define COMMS_UART Serial2  // UART between Main-Motor

#    define ENCODER_AXIS0_PIN 32
#    define ENCODER_AXIS1_PIN 33
#    define ENCODER_AXIS2_PIN 25
#    define ENCODER_AXIS3_PIN 26

#    define CAN_TX 13
#    define CAN_RX 14

#    define PIN_VDIV_BATT 39
#    define PIN_VDIV_12V 36
#    define PIN_VDIV_5V 34
#    define PIN_VDIV_3V3 35

#    define MOTOR_AMOUNT 4


#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)

//------------------------------------------------------------------------------------------------//
//   Feather ESP32 (URC 2025, Socket V1)
//------------------------------------------------------------------------------------------------//

#    define COMMS_UART Serial1  // UART between Main-Motor

#    define ENCODER_AXIS0_PIN 33
#    define ENCODER_AXIS1_PIN 26
#    define ENCODER_AXIS2_PIN 25
#    define ENCODER_AXIS3_PIN 4

#    define CAN_TX 13
#    define CAN_RX 12

#    define LYNX_TX 22
#    define LYNX_RX 20

#    define PIN_VDIV_BATT 39
#    define PIN_VDIV_12V 36
#    define PIN_VDIV_5V 34
#    define PIN_VDIV_3V3 35

#    define AX0_CS 15

#    define MOTOR_AMOUNT 4


#elif defined(CORE_TEENSY)

//------------------------------------------------------------------------------------------------//
//   Teensy 4.x (URC 2024)
//------------------------------------------------------------------------------------------------//

//------//
// Pins //
//------//

#    define PIN_AS5047P_1_CS 10
#    define PIN_AS5047P_2_CS 37
#    define PIN_AS5047P_3_CS 36
#    define PIN_AXIS_0_PWM 19

//-----------//
// Constants //
//-----------//

#    define COMMS_UART Serial3

#    define LSS_SERIAL Serial7

#    define AS5047P_CUSTOM_SPI_BUS_SPEED 10'000'000


#endif
