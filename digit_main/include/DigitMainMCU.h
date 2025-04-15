/**
 * @file DigitMainMCU.h
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Digit PCB; controls end effector, wrist, and now FAERIE; attaches to end of arm
 *
 */
#pragma once


#if defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)

//------------------------------------------------------------------------------------------------//
//   DOIT ESP32 Devkit V1 (URC 2025, Digit+FAERIE V2)
//------------------------------------------------------------------------------------------------//

// Comms

#   define CAN_RX 27
#   define CAN_TX 14

// Linear actuator

#   define LINAC_RIN 19
#   define LINAC_FIN 18

// End Effector

#   define MOTOR_IN1 33
#   define MOTOR_IN2 25
#   define MOTOR_FAULT 4

#   define LASER_NMOS 23  // same for faerie

// ADC

#   define PIN_VDIV_5V 35
#   define PIN_VDIV_12V 36
#   define PIN_VDIV_BATT 39

// FAERIE

#   define SPARK_PWM 26

// Misc

#   define PIN_NEOPIXEL 13

// LSS

#   define LSS_SERIAL Serial2


#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32_V2)

//------------------------------------------------------------------------------------------------//
//   Feather ESP32 (URC 2025, Digit V1)
//------------------------------------------------------------------------------------------------//

// Comms

#   define CAN_RX 14
#   define CAN_TX 32

// Linear actuator

#   define LINAC_RIN 12
#   define LINAC_FIN 13

// End Effector

#   define MOTOR_FAULT 5
#   define MOTOR_IN2 19
#   define MOTOR_IN1 21

#   define LASER_NMOS 25

// ADC

#   define PIN_VDIV_5V 34
#   define PIN_VDIV_12V 39
#   define PIN_VDIV_BATT 36

// Misc

#   define MCU_DEBUG 4

// Lynxmotion Smart Servo

#   define LSS_SERIAL Serial1


#elif defined(CORE_TEENSY)

//------------------------------------------------------------------------------------------------//
//   Teensy 4.x (URC 2024)
//------------------------------------------------------------------------------------------------//

//------//
// Pins //
//------//

// DC on/off control for laser on end effector
#    define PIN_LASER 8
// PWM control for REV motor that opens/closes end effector
#    define PIN_EF_MOTOR 19

//-----------//
// Constants //
//-----------//

#    define COMMS_UART Serial1


#endif
