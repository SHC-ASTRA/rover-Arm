/**
 * @file ARM.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Arm
 *
 */
#pragma once


#if !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)

//------------------------------------------------------------------------------------------------//
//   DOIT ESP32 Devkit V1 (URC 2025, Socket V2)
//------------------------------------------------------------------------------------------------//

#    define CAN_TX 13
#    define CAN_RX 14


#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)

//------------------------------------------------------------------------------------------------//
//   Feather ESP32 (URC 2025, Socket V1)
//------------------------------------------------------------------------------------------------//

#    define CAN_TX 13
#    define CAN_RX 12


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
