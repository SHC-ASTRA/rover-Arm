/**
 * @file main.cpp
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @author David Sharpe (ds0196@uah.edu)
 * @brief Digit ESP32; controls End Effector and Wrist
 *
 */

//------------//
//  Includes  //
//------------//

#include <Arduino.h>
#include <LSS.h>

#include "AstraMisc.h"
#include "AstraVicCAN.h"
#include "project/DIGIT.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK


//---------------------//
//  Component classes  //
//---------------------//

LSS topLSS = LSS(LSS_TOP_ID);
LSS bottomLSS = LSS(LSS_BOTTOM_ID);


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

unsigned long lastCtrlCmd = millis();

uint32_t lastFault = 0;


//--------------//
//  Prototypes  //
//--------------//


//------------------------------------------------------------------------------------------------//
//  Setup
//------------------------------------------------------------------------------------------------//
//
//
//------------------------------------------------//
//                                                //
//      ////////    //////////    //////////      //
//    //                //        //        //    //
//    //                //        //        //    //
//      //////          //        //////////      //
//            //        //        //              //
//            //        //        //              //
//    ////////          //        //              //
//                                                //
//------------------------------------------------//
void setup() {
    //--------//
    //  Pins  //
    //--------//

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    // Laser
    pinMode(LASER_NMOS, OUTPUT);
    digitalWrite(LASER_NMOS, LOW);

    // Linear actuator
    pinMode(LINAC_RIN, OUTPUT);
    pinMode(LINAC_FIN, OUTPUT);
    digitalWrite(LINAC_RIN, LOW);
    digitalWrite(LINAC_FIN, LOW);

    // End Effector motor
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);
    pinMode(MOTOR_FAULT, INPUT);  // External pull-up resistor
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);


    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);

    if(ESP32Can.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX))
        Serial.println("CAN bus started!");
    else
        Serial.println("CAN bus failed!");


    //-----------//
    //  Sensors  //
    //-----------//


    //--------------------//
    //  Misc. Components  //
    //--------------------//

    LSS::initBus(LSS_SERIAL, LSS_DefaultBaud);

    // LSS Settings from last year's Arm
    topLSS.setMaxSpeed(100);
    bottomLSS.setMaxSpeed(100);

    // Complete LSS configuration
    topLSS.reset();
    bottomLSS.reset();

    // Wait for LSS reboot
    delay(2000);
}


//------------------------------------------------------------------------------------------------//
//  Loop
//------------------------------------------------------------------------------------------------//
//
//
//-------------------------------------------------//
//                                                 //
//    /////////      //            //////////      //
//    //      //     //            //        //    //
//    //      //     //            //        //    //
//    ////////       //            //////////      //
//    //      //     //            //              //
//    //       //    //            //              //
//    /////////      //////////    //              //
//                                                 //
//-------------------------------------------------//
void loop() {
    //----------//
    //  Timers  //
    //----------//
#ifdef BLINK
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
#endif

    // Motor control safety timeout
    if (millis() - lastCtrlCmd > 2000) {
        lastCtrlCmd = millis();
        // Stop LSS
        topLSS.wheelRPM(0);
        bottomLSS.wheelRPM(0);
        // Stop EF motor
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
        // Stop lin ac
        digitalWrite(LINAC_RIN, LOW);
        digitalWrite(LINAC_FIN, LOW);
#ifdef DEBUG
        Serial.println("Safety timeout");
#endif
    }

    // EF motor controller fault monitor
    if (millis() - lastFault > 1000 && digitalRead(MOTOR_FAULT) == LOW) {
        lastFault = millis();  // Always check unless there has been a fault <1 second ago
        Serial.println("EF Motor fault detected: over-current, over-temperature, or under-voltage.");
        // Stop EF motor
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
    }


    //-------------//
    //  CAN Input  //
    //-------------//
    if(vicCAN.readCan()) {
        const uint8_t commandID = vicCAN.getCmdId();
        static std::vector<double> canData;
        vicCAN.parseData(canData);

        Serial.print("VicCAN: ");
        Serial.print(commandID);
        Serial.print("; ");
        if (canData.size() > 0) {
            for (const double& data : canData) {
                Serial.print(data);
                Serial.print(", ");
            }
        }
        Serial.println();


        // General Misc

        /**/ if (commandID == CMD_PING) {
            vicCAN.respond(1);  // "pong"
            Serial.println("Received ping over CAN");
        }
        else if (commandID == CMD_B_LED) {
            if (canData.size() == 1) {
                if (canData[0] == 0)
                    digitalWrite(LED_BUILTIN, false);
                if (canData[0] == 1)
                    digitalWrite(LED_BUILTIN, true);
            }
        }

        // Misc Physical Control

        else if (commandID == CMD_LASER_CTRL) {
            if (canData.size() == 1) {
                if (canData[0] == 0) {
#ifdef DEBUG
                    Serial.println("Laser off");
#endif
                    digitalWrite(LASER_NMOS, LOW);
                }
                else if (canData[0] == 1) {
#ifdef DEBUG
                    Serial.println("Laser on");
#endif
                    digitalWrite(LASER_NMOS, HIGH);
                }
            }
        }
    }


    //------------------//
    //  UART/USB Input  //
    //------------------//
    //
    //
    //-------------------------------------------------------//
    //                                                       //
    //      /////////    //\\        ////    //////////      //
    //    //             //  \\    //  //    //        //    //
    //    //             //    \\//    //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //      /////////    //            //    //////////      //
    //                                                       //
    //-------------------------------------------------------//
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');

        input.trim();                   // Remove preceding and trailing whitespace
        std::vector<String> args = {};  // Initialize empty vector to hold separated arguments
        parseInput(input, args, ',');   // Separate `input` by commas and place into args vector
        args[0].toLowerCase();          // Make command case-insensitive
        String command = args[0];       // To make processing code more readable

        // Backwards compatibility with last year's ROS2 code
        if (command == "digit") {
            // Remove first argument, which is "digit" to tell socket teensy to redirect to digit
            args.erase(args.begin());
            // Our command is not "digit", but what comes after it
            command = args[0];
            command.toLowerCase();
        }

        //--------//
        //  Misc  //
        //--------//
        /**/ if (command == "ping") {
            Serial.println("pong");
        }

        else if (command == "time") {
            Serial.println(millis());
        }

        else if (command == "led") {
            if (args[1] == "on")
                digitalWrite(LED_BUILTIN, HIGH);
            else if (args[1] == "off")
                digitalWrite(LED_BUILTIN, LOW);
            else if (args[1] == "toggle") {
                ledState = !ledState;
                digitalWrite(LED_BUILTIN, ledState);
            }
        }

        else if (command == "can_relay_tovic") {
            vicCAN.relayFromSerial(args);
        }

        //-----------//
        //  Sensors  //
        //-----------//

        //----------//
        //  Motors  //
        //----------//

        else if (command == "ctrl") {
            lastCtrlCmd = millis();

            if (args[1] == "lin_ac") {
                if (args[2] == "1") {
                    digitalWrite(LINAC_RIN, LOW);
                    digitalWrite(LINAC_FIN, HIGH);
                } else if (args[2] == "0") {
                    digitalWrite(LINAC_RIN, HIGH);
                    digitalWrite(LINAC_FIN, HIGH);
                } else if (args[2] == "-1") {
                    digitalWrite(LINAC_RIN, HIGH);
                    digitalWrite(LINAC_FIN, LOW);
                }
            }
            else if (args[1] == "lss") {
                if (args[2] == "reset") {
                    topLSS.reset();
                    bottomLSS.reset();
                } else if (args[2] == "manual") {
                    // For now, just take raw speeds for the two servos.
                    // We can figure out the math for yaw/rotation at the same time later...
                    topLSS.wheelRPM(args[3].toInt());
                    bottomLSS.wheelRPM(args[4].toInt());
                } else if (args[2] == "ik") {
                    Serial.println("IK not implemented yet");
                    // Will need math to figure out what yaw angle the wrist is currently at and how
                    // to get to the target angle with the differential
                }
            }
            else if (args[1] == "ef") {
                if (args[2] == "1") {
                    digitalWrite(MOTOR_IN1, HIGH);
                    digitalWrite(MOTOR_IN2, LOW);
                } else if (args[2] == "0") {
                    digitalWrite(MOTOR_IN1, HIGH);
                    digitalWrite(MOTOR_IN2, HIGH);
                } else if (args[2] == "-1") {
                    digitalWrite(MOTOR_IN1, LOW);
                    digitalWrite(MOTOR_IN2, HIGH);
                }
            }

        } else if (command == "laser") {
            if (args[1] == "0") {
                digitalWrite(LASER_NMOS, LOW);
            } else if (args[1] == "1") {
                digitalWrite(LASER_NMOS, HIGH);
            }

        }
    }
}


//------------------------------------------------------------------------------------------------//
//  Function definitions
//------------------------------------------------------------------------------------------------//
//
//
//----------------------------------------------------//
//                                                    //
//    //////////    //          //      //////////    //
//    //            //\\        //    //              //
//    //            //  \\      //    //              //
//    //////        //    \\    //    //              //
//    //            //      \\  //    //              //
//    //            //        \\//    //              //
//    //            //          //      //////////    //
//                                                    //
//----------------------------------------------------//
