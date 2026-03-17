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
#include <math.h>

#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
#    include <SPI.h>    // Fixes compilation issue with Adafruit BusIO
#    include <ESP32Servo.h>
#    include <Adafruit_SHT31.h>  // adafruit/Adafruit SHT31 Library
#endif

#include "AstraMisc.h"
#include "AstraNP.h"
#include "AstraVicCAN.h"
#include "DigitMainMCU.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

#define LSS_GEAR_RATIO 2

#define LSS_TOP_ID 1
#define LSS_BOTTOM_ID 2

#define REV_PWM_MIN 1000  // us  -1.0 duty
#define REV_PWM_MAX 2000  // us  1.0 duty


//---------------------//
//  Component classes  //
//---------------------//

LSS topLSS = LSS(LSS_TOP_ID);
LSS bottomLSS = LSS(LSS_BOTTOM_ID);

AstraNeoPixel np(PIN_NEOPIXEL);

#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2

// Now controls SCABBARD NEO-550 motor
Servo neo550;

// Faerie HUM/TEMP sensor (in SCABBARD)
Adafruit_SHT31 sht;

#endif


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

long lastCtrlCmd = millis();

long lastFault = 0;

long lastWristCtrl = 0;  // ms

bool isWristCtrlIK = false;  // Is IK controlling wrist now?
float wristIKYawGoal = 0;  // degrees; Goal for wristYaw from IK
float wristIKRollGoal = 0;  // degrees; Goal for wristRoll from IK

float lastWristYaw = 0;  // degrees; Last known wrist yaw angle
float lastWristRoll = 0;  // degrees; Last known wrist roll angle

int wristManYawDir = 0;  // Manual wrist yaw direction - 1, 0, -1
int wristManRollDir = 0;  // Manual wrist roll direction - 1, 0, -1

float wristYawRPM = 0;  // Wrist yaw rpm
float wristRollRPM = 0;  // Wrist roll rpm

long lastFeedback = 0;  // ms
long lastVoltRead = 0;
long lastDataSend = 0;
long lastNP = 0;

#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
// Shake mode variables

const float SHAKEOPTIONS[5] = {0.1, 0.2, 0.3, 0.4, 0.5};
const uint32_t SHAKEINTERVAL = 250;   // Shake interval
const uint32_t SHAKEDURATION = 2500;  // How long to shake
uint32_t shakeStart = 0;              // millis value when shaking started
uint32_t lastShake = 0;               // Last millis value of shake update
bool shakeMode = false;               // Whether or not currently shaking
int shakeDir = 1;                     // Positive or negative to shake in open or close dir

#endif


//--------------//
//  Prototypes  //
//--------------//

void stopEverything();
void efCtrl(int dir);
float clamp_angle(float angle);


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

    np.writeColor(COLOR_SETUP_START);

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
#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
#else
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
#endif


    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);

    if(ESP32Can.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX)) {
        Serial.println("CAN bus started!");
    } else {
        Serial.println("CAN bus failed!");
        np.addStatus(STATUS_CAN_NOCONN, 30);
    }


    //-----------//
    //  Sensors  //
    //-----------//

    np.writeColor(COLOR_SETUP_SENSORS);

#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
    if (sht.begin(0x44)) {  // HUM/Temp
        Serial.println("SHT31 initialized.");
    } else {
        Serial.println("Couldn't find SHT31!");
        // np.addStatus(STATUS_SHT_NOCONN, 30);
    }
#endif


    //--------------------//
    //  Misc. Components  //
    //--------------------//

    LSS::initBus(LSS_SERIAL, LSS_DefaultBaud);

    // Check for a connection to the Lynxmotion servos
    Serial.println(topLSS.getVoltage());
    if (topLSS.getLastCommStatus() != LSS_CommStatus_ReadSuccess)
        Serial.println("Top LSS not found!");
    Serial.println(bottomLSS.getVoltage());
    if (bottomLSS.getLastCommStatus() != LSS_CommStatus_ReadSuccess)
        Serial.println("Bottom LSS not found!");

    /* LSS Configuration:
        A persistent configuration is expected to be saved on both LSS servos.
        - Max speed: 10 deg/s (1 degree / 175 ms)
        - Origin offset: unique to either servo's mounting
        - LED color: blue for one of them, white for the other (i forgor)
        - ID: 1 for top, 2 for bottom
    */

#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
    // SCABBARD NEO-550 motor
    neo550.attach(SPARK_PWM, REV_PWM_MIN, REV_PWM_MAX);
#endif

    np.writeColor(COLOR_SETUP_DONE);

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

    // IK Angles
    if (millis() - lastFeedback > 500) {
        lastFeedback = millis();
        vicCAN.send(CMD_ARM_ENCODER_ANGLES, lastWristYaw, lastWristRoll);
    }

    // LSS RPM
    if (millis() - lastFeedback > 500) {
        lastFeedback = millis();
        vicCAN.send(58, wristYawRPM, wristRollRPM); // TODO: Set CMD_ARM_RPM_FEEDBACK to 58
    }

    // LSS Current and Temp
    if (millis() - lastFeedback > 500) {
        lastFeedback = millis();
        vicCAN.send(59, topLSS.getCurrent(), bottomLSS.getCurrent(), 
                                      topLSS.getTemperature(), bottomLSS.getTemperature()); // TODO: Set CMD_LSS_FEEDBACK to 59
    }

#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
    // Telemetry
    if (millis() - lastDataSend >= 1000) {
        lastDataSend = millis();

        float temp, hum;
        sht.readBoth(&temp, &hum);
        
        vicCAN.send(57, temp, hum);
    }

    // SCABBARD Shake
    if (shakeMode && millis() - lastShake >= SHAKEINTERVAL) {
        lastShake = millis();

        unsigned ind = rand() % 5;  // 0-4 inclusive, seeded by command

        if (ind < 0 || ind > 4)
            ind = 0;
        neo550.write(0);

        // Don't shake for longer than SHAKEDURATION
        if (shakeStart + SHAKEDURATION <= millis()) {
            shakeMode = false;
            neo550.write(0);
        }
    }
#endif

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
#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
        analogWrite(MOTOR_IN1, 0);
        analogWrite(MOTOR_IN2, 0);
#else
        digitalWrite(MOTOR_IN1, LOW);
        digitalWrite(MOTOR_IN2, LOW);
#endif
    }

    // Neopixel status update
    if (millis() - lastNP > 50) {
        lastNP = millis();
        np.update();
    }

    // Wrist Control
    if (millis() - lastWristCtrl > 100) {
        lastWristCtrl = millis();

        float topLSSAngle = topLSS.getPosition() / 10.0;
        float bottomLSSAngle = bottomLSS.getPosition() / 10.0;

        lastWristYaw = clamp_angle(topLSSAngle - bottomLSSAngle) / 2.0;
        lastWristRoll = (topLSSAngle + bottomLSSAngle) / (2 * LSS_GEAR_RATIO);

        float topRPM = topLSS.getSpeedRPM();
        float bottomRPM = bottomLSS.getSpeedRPM();

        wristYawRPM  = (topRPM - bottomRPM) / 2.0;
        wristRollRPM = (topRPM + bottomRPM) / (2 * LSS_GEAR_RATIO);

        // IK Control
        if (isWristCtrlIK) {
            float k = 1;  // Mechanical constant

            float topTarget = LSS_GEAR_RATIO * wristIKRollGoal + (wristIKYawGoal / (2 * k));
            float bottomTarget = LSS_GEAR_RATIO * wristIKRollGoal - (wristIKYawGoal / (2 * k));

            topLSS.move(topTarget * 10.0);
            bottomLSS.move(bottomTarget * 10.0);
        }
        // Manual Control
        else {
            if (wristManRollDir == 0 && wristManYawDir == 0) {  // Stop as soon as possible if requested
                topLSS.wheel(0);
                bottomLSS.wheel(0);
            } else {  // Still moving
                // Yaw bounds check
                if ((lastWristYaw < -70 && wristManYawDir == 1) || (lastWristYaw > 70 && wristManYawDir == -1))
                    wristManYawDir = 0;

                int yawSpeed = 0;
                int rollSpeed = 0;

                if (wristManYawDir == 1)
                    yawSpeed = -20;
                else if (wristManYawDir == -1)
                    yawSpeed = 20;

                if (wristManRollDir == 1)
                    rollSpeed = -20 * LSS_GEAR_RATIO;
                else if (wristManRollDir == -1)
                    rollSpeed = 20 * LSS_GEAR_RATIO;

                topLSS.wheel(yawSpeed + rollSpeed);
                bottomLSS.wheel(-yawSpeed + rollSpeed);
            }
        }
    }

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
        if (commandID == CMD_PING) {
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

        // REV

#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
        else if (commandID == CMD_REV_STOP) {
            lastCtrlCmd = millis();
            neo550.write((REV_PWM_MIN + REV_PWM_MAX) / 2);
        }

        else if (commandID == CMD_REV_SET_DUTY) {
            if (canData.size() == 1) {
                lastCtrlCmd = millis();
                int value = map_d(canData[0] / 100.0, -1.0, 1.0, REV_PWM_MIN, REV_PWM_MAX);
                neo550.writeMicroseconds(value);
                Serial.print("Setting REV duty to ");
                Serial.println(value);
            }
        }
#endif

        // Misc Physical Control

#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
        else if (commandID == CMD_DCMOTOR_CTRL) {
            if (canData.size() == 1) {
                lastCtrlCmd = millis();
                efCtrl(canData[0]);
            }
        }
#endif

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

        else if (commandID == CMD_LSS_RESET) {
            lastCtrlCmd = millis();
            topLSS.reset();
            bottomLSS.reset();
        }

        // Submodule Specific

        else if (commandID == CMD_ARM_IK_CTRL) {
            if (canData.size() == 2) {
                lastCtrlCmd = millis();

                wristIKYawGoal = canData[0];
                wristIKRollGoal = canData[1];
                isWristCtrlIK = true;
            }
        }

        else if (commandID == CMD_DIGIT_LINAC_CTRL) {
            if (canData.size() == 1) {
                lastCtrlCmd = millis();

                if (canData[0] == 1) {  // Extend
                    digitalWrite(LINAC_RIN, HIGH);
                    digitalWrite(LINAC_FIN, LOW);
                } else if (canData[0] == 0) {  // Stop
                    digitalWrite(LINAC_RIN, LOW);
                    digitalWrite(LINAC_FIN, LOW);
                } else if (canData[0] == -1) {  // Retract
                    digitalWrite(LINAC_RIN, LOW);
                    digitalWrite(LINAC_FIN, HIGH);
                }
            }
        }

        else if (commandID == CMD_ARM_MANUAL) {  // Wrist roll + yaw
            if (canData.size() == 2) {
                lastCtrlCmd = millis();

                // Set globals for 10 Hz controller timer to take care of
                wristManYawDir = canData[0];
                wristManRollDir = canData[1];
                isWristCtrlIK = false;  // Disable IK if doing manual control

                if (wristManYawDir == 0 && wristManRollDir == 0) {
                    topLSS.wheel(0);
                    bottomLSS.wheel(0);
                }
            }
        }

#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
        else if (commandID == CMD_FAERIE_SKAKE) {  // TODO: fix typo :(
            if (canData.size() == 1 && (canData[0] == -1 || canData[0] == 1)) {
                lastCtrlCmd = millis();

                shakeMode = true;
                shakeDir = canData[0];
                lastShake = 0;
                shakeStart = millis();

                // Seed rand() for random duty cycles
                srand(millis());
            }
        }

        else if (commandID == 42) {  // New linear actuator for FAERIE
            if (canData.size() == 1) {
                lastCtrlCmd = millis();
                // Defeat the evil analogWrite()
                // pinMode(MOTOR_IN1, OUTPUT);
                // pinMode(MOTOR_IN2, OUTPUT);
                if (canData[0] <= -1) {
                    digitalWrite(MOTOR_IN1, LOW);
                    digitalWrite(MOTOR_IN2, HIGH);
                } else if (canData[0] == 0) {
                    digitalWrite(MOTOR_IN1, LOW);
                    digitalWrite(MOTOR_IN2, LOW);
                } else if (canData[0] >= 1) {
                    digitalWrite(MOTOR_IN1, HIGH);
                    digitalWrite(MOTOR_IN2, LOW);
                }
            }
        }
#endif
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
                    Serial.print('k');
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
#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
    analogWrite(MOTOR_IN1, 0);
    analogWrite(MOTOR_IN2, 0);
#else
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
#endif
    // Stop lin ac
    digitalWrite(LINAC_RIN, LOW);
    digitalWrite(LINAC_FIN, LOW);
    // Turn off laser
    digitalWrite(LASER_NMOS, LOW);
#ifndef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
    // Stop SCABBARD NEO-550 motor
    neo550.write((REV_PWM_MIN + REV_PWM_MAX) / 2);
#endif
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

float clamp_angle(float angle) {
    // https://stackoverflow.com/a/11498248
    angle = fmod(angle + 180, 360);
    if (angle < 0)
        angle += 360;
    return angle - 180;
}