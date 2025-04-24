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
#include "DigitMainMCU.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

#define LSS_GEAR_RATIO 3

#define LSS_TOP_ID 1
#define LSS_BOTTOM_ID 2


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

uint32_t lastWristYawIter = 0;  // ms

int wristYaw = 0;  // degrees; Current yaw angle of wrist
int wristYawDir = 0;  // Direction of wrist yaw: 1 = Close, 0 = Stop, -1 = Open
int wristRollDir = 0;

bool isWristYawIK = false;  // Is IK controlling yaw now?
int wristYawIKGoal = 0;  // degrees; Goal for wristYaw from IK
int timeToGoal = 0;  // ms

unsigned long lastFeedback = 0;  // ms
unsigned long lastVoltRead = 0;


//--------------//
//  Prototypes  //
//--------------//

void stopEverything();
void efCtrl(int dir);


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
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);


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

    // 1 degree / 175 ms
    topLSS.setMaxSpeed(100);
    bottomLSS.setMaxSpeed(100);

    // Complete LSS configuration
    // topLSS.reset();
    // bottomLSS.reset();

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
        stopEverything();
#ifdef DEBUG
        Serial.println("Safety timeout");
#endif
    }

    if (millis() - lastFeedback > 500) {
        lastFeedback = millis();
        vicCAN.send(CMD_ARM_ENCODER_ANGLES, wristYaw);  // Currently just 0
    }

    if (millis() - lastVoltRead > 1000) {
        lastVoltRead = millis();
        float vBatt = convertADC(analogRead(PIN_VDIV_BATT), 10, 2.21);
        float v12 = convertADC(analogRead(PIN_VDIV_12V), 10, 3.32);
        float v5 = convertADC(analogRead(PIN_VDIV_5V), 10, 10);

        vicCAN.send(CMD_POWER_VOLTAGE, vBatt * 100, v12 * 100, v5 * 100);
    }

    // EF motor controller fault monitor
    if (millis() - lastFault > 1000 && digitalRead(MOTOR_FAULT) == LOW) {
        lastFault = millis();  // Always check unless there has been a fault <1 second ago
        Serial.println("EF Motor fault detected: over-current, over-temperature, or under-voltage.");
        // Stop EF motor
        analogWrite(MOTOR_IN1, 0);
        analogWrite(MOTOR_IN2, 0);
    }

    // if (millis() - lastWristYawIter > 175 && (isWristYawIK || wristYawDir != 0)) {
    //     lastWristYawIter = millis();

    //     if (isWristYawIK && wristYaw != wristYawIKGoal) {  // IK Yaw Control
    //         // TODO: decimal?
    //         if (wristYaw < wristYawIKGoal) {
    //             wristYaw += 1;
    //             topLSS.moveRelative(-20);
    //             bottomLSS.moveRelative(20);
    //         } else if (wristYaw > wristYawIKGoal) {
    //             wristYaw -= 1;
    //             topLSS.moveRelative(20);
    //             bottomLSS.moveRelative(-20);
    //         }
    //     } else if (!isWristYawIK && wristYawDir != 0) {  // Manual Yaw Control
    //         if (wristYawDir == 1) {
    //             wristYaw += 1;
    //             topLSS.moveRelative(-20);
    //             bottomLSS.moveRelative(20);
    //         } else if (wristYawDir == -1) {
    //             wristYaw -= 1;
    //             topLSS.moveRelative(20);
    //             bottomLSS.moveRelative(-20);
    //         }
    //     }
    // }


    //-------------//
    //  CAN Input  //
    //-------------//
    if(vicCAN.readCan()) {
        const uint8_t commandID = vicCAN.getCmdId();
        static std::vector<double> canData;
        vicCAN.parseData(canData);

        Serial.print(millis());
        Serial.print(": ");
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

        else if (commandID == CMD_DCMOTOR_CTRL) {
            if (canData.size() == 1) {
                lastCtrlCmd = millis();
                efCtrl(canData[0]);
            }
        }

        else if (commandID == CMD_LASER_CTRL) {
            if (canData.size() == 1) {
                lastCtrlCmd = millis();
                if (canData[0] == 0) {
                    digitalWrite(LASER_NMOS, LOW);
                }
                else if (canData[0] == 1) {
                    digitalWrite(LASER_NMOS, HIGH);
                }
            }
        }

        else if (commandID == CMD_ARM_IK_TTG) {
            if (canData.size() == 1) {
                timeToGoal = canData[0];
            }
        }

        else if (commandID == 35) {  // Wrist rotate
            if (canData.size() == 1 && (!isWristYawIK && wristYawDir == 0)) {
                lastCtrlCmd = millis();

                wristRollDir = canData[0];

                if (canData[0] == 1) {
                    topLSS.wheel(-20 * LSS_GEAR_RATIO);
                    bottomLSS.wheel(-20 * LSS_GEAR_RATIO);
                } else if (canData[0] == 0 && wristYawDir == 0) {
                    topLSS.wheel(0);
                    bottomLSS.wheel(0);
                } else if (canData[0] == -1) {
                    topLSS.wheel(20 * LSS_GEAR_RATIO);
                    bottomLSS.wheel(20 * LSS_GEAR_RATIO);
                }
            }
        }

        else if (commandID == 36) {  // Wrist yaw
            if (canData.size() == 2) {
                lastCtrlCmd = millis();
                // isWristYawIK = static_cast<bool>(canData[0]);
                // wristYawDir = canData[1];
                // wristYawIKGoal = canData[1];

                if (canData[0] == 0) {  // Manual control
                    wristYawDir = canData[1];

                    if (canData[1] == 1) {
                        topLSS.wheel(-20);
                        bottomLSS.wheel(20);
                    } else if (canData[1] == 0 && wristRollDir == 0) {
                        topLSS.wheel(0);
                        bottomLSS.wheel(0);
                    } else if (canData[1] == -1) {
                        topLSS.wheel(20);
                        bottomLSS.wheel(-20);
                    }
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
        parseInput(input, args);   // Separate `input` by commas and place into args vector
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

        else if (args[0] == "can_relay_mode") {
            if (args[1] == "on") {
                vicCAN.relayOn();
            } else if (args[1] == "off") {
                vicCAN.relayOff();
            }
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
                    topLSS.moveRelative(args[3].toInt());
                    bottomLSS.moveRelative(args[4].toInt());
                    Serial.print('j');
                    Serial.print(args[3].toInt());
                    Serial.print(' ');
                    Serial.println(args[4].toInt());
                } else if (args[2] == "ik") {
                    Serial.println("IK not implemented yet");
                    // Will need math to figure out what yaw angle the wrist is currently at and how
                    // to get to the target angle with the differential
                }
            }
            else if (args[1] == "ef") {
                efCtrl(args[2].toInt());
            }

        } else if (command == "laser") {
            if (args[1] == "0") {
                digitalWrite(LASER_NMOS, LOW);
            } else if (args[1] == "1") {
                digitalWrite(LASER_NMOS, HIGH);
            }

        }

        else if (command == "stop") {
            stopEverything();
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

void stopEverything() {
    // Stop LSS
    topLSS.wheel(0);
    bottomLSS.wheel(0);
    // topLSS.limp();
    // bottomLSS.limp();
    // Stop EF motor
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
    // Stop lin ac
    digitalWrite(LINAC_RIN, LOW);
    digitalWrite(LINAC_FIN, LOW);
    // Turn off laser
    digitalWrite(LASER_NMOS, LOW);
}

void efCtrl(int dir) {
    if (dir == 1) {  // Close
        analogWrite(MOTOR_IN1, 225);
        analogWrite(MOTOR_IN2, 0);
    } else if (dir == 0) {  // Stop 
        analogWrite(MOTOR_IN1, 50);
        analogWrite(MOTOR_IN2, 50);
    } else if (dir == -1) {  // Open
        analogWrite(MOTOR_IN1, 0);
        analogWrite(MOTOR_IN2, 225);
    }
}
