/**
 * @file Main.cpp
 * @author Charles Marmann (cmm0077@uah.edu)
 * @author David Sharpe (ds0196@uah.edu)
 * @author Jack Schumacher (js0342@uah.edu)
 * @brief Controls REV motors on ASTRA's Arm submodule
 *
 */

//------------//
//  Includes  //
//------------//

#include <Arduino.h>


#include <cmath>

#include "AstraMisc.h"
#include "AstraMotors.h"


// Project header
#include "project/ARM.h"

// Temporary Need to Remove
#include "project/CORE.h"

// REV CAN
#ifdef OLD_ASTRACAN_ENABLE
#    include "AstraCAN.h"
#else
#    include "AstraREVCAN.h"
#endif


//------------//
//  Settings  //
//------------//

// #define DEBUG_STATUS

// #ifdef DEBUG
// #    define COMMS_UART Serial
// #endif


//---------------------//
//  Component classes  //
//---------------------//

// AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float
// setMaxDuty)
AstraMotors MotorAxis1(MOTOR_ID_A1, sparkMax_ctrlType::kDutyCycle, true, 1000, 1.0);   // Back Right
AstraMotors MotorAxis2(MOTOR_ID_A2, sparkMax_ctrlType::kDutyCycle, true, 1000, 1.0);   // Back Right
AstraMotors MotorAxis3(MOTOR_ID_A3, sparkMax_ctrlType::kDutyCycle, true, 1000, 1.0);   // Back Right

AstraMotors* motorList[] = {&MotorAxis1, &MotorAxis2, &MotorAxis3};

//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;
unsigned long lastHB = 0;
int heartBeatNum = 1;
unsigned long lastCtrlCmd = 0;
unsigned long lastMotorStatus = 0;
unsigned long lastMotorFeedback = 0;
bool safetyOn = true;

//--------------//
//  Prototypes  //
//--------------//

bool setAxisDeg(int axis, int degrees, int timeout, bool rel_abs);      // set what degree the axis is trying to go to
void setAxisSpeeds(float A1Speed, float A2Speed, float A3Speed);                                 // set speed at which an axis moves
void Brake(bool enable);
void loopHeartbeats();
void safety_timeout();
float getDriveSpeed();
void updateMotorStatus();
void motorFeedback();
void Stop();


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

    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);              // Communication between motor MCU and Type-C Port
    COMMS_UART.begin(COMMS_UART_BAUD);      // Communication between both the main and motor microcontroller

    //-----------//
    //  Sensors  //
    //-----------//

    //--------------------//
    //  Misc. Components  //
    //--------------------//

    // Setup CAN
    if (ESP32Can.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX)) 
    {
        COMMS_UART.println("CAN bus started!");
    } 
    else 
    {
        COMMS_UART.println("CAN bus failed!");
    }
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

    // Blink the LED
    if (millis() - lastBlink >= 1000) 
    {
        lastBlink = millis();
        ledState = !ledState;
        if (ledState)
            digitalWrite(LED_BUILTIN, HIGH);
        else
            digitalWrite(LED_BUILTIN, LOW);
    }

    // Accelerate motors; update the speed for all motors

    // Heartbeat for REV motors
    if (millis() - lastHB >= 3)
    {
        sendHeartbeat(ESP32Can, heartBeatNum);
        lastHB = millis();
        heartBeatNum++;
        if (heartBeatNum > 4)
        {
            heartBeatNum = 1;
        }
    }

    if (millis() - lastMotorFeedback >= 2000) // Change to 1000?
    {
        lastMotorFeedback = millis();
        motorFeedback();
    }

    // Safety timeout
    safety_timeout();

    // Motor status debug printout
#if defined(DEBUG2)
    if (millis() - lastMotorStatus > 500) {
        lastMotorStatus = millis();

        // Status 1
        Serial.print(millis() - Motor2.status1.timestamp);
        Serial.print(" ms ago: ");
        Serial.print(Motor2.status1.motorTemperature);
        Serial.print(" *C; ");
        Serial.print(Motor2.status1.busVoltage);
        Serial.print(" V; ");
        Serial.print(Motor2.status1.outputCurrent);
        Serial.print(" A; ");
        Serial.print(Motor2.status1.sensorVelocity);
        Serial.println(" RPM");
        
        // Status 2
        Serial.print(millis() - Motor2.status2.timestamp);
        Serial.print(" ms ago: ");
        Serial.print(Motor2.status2.sensorPosition);
        Serial.println(" rotations");
    }
#endif

    //-------------//
    //  CAN Input  //
    //-------------//

    static CanFrame rxFrame;
    if (ESP32Can.readFrame(rxFrame, 1)) {
        // Decode the ID

        uint32_t msgId = rxFrame.identifier;
        // Pull out device ID and API ID
        uint8_t deviceId = msgId & 0x3F;
        uint32_t apiId = (msgId >> 6) & 0x3FF;

        if (apiId == 0x60) {
            for (int i = 0; i < MOTOR_AMOUNT; i++) {
                if (deviceId == motorList[i]->getID()) {
                    motorList[i]->parseStatus0(rxFrame.data);
                    break;
                }
            }
        } else if (apiId == 0x61) {  // Status 1
            for (int i = 0; i < MOTOR_AMOUNT; i++) {
                if (deviceId == motorList[i]->getID()) {
                    motorList[i]->parseStatus1(rxFrame.data);
                    break;
                }
            }
        } else if (apiId == 0x62) {  // Status 2
            for (int i = 0; i < MOTOR_AMOUNT; i++) {
                if (deviceId == motorList[i]->getID()) {
                    motorList[i]->parseStatus2(rxFrame.data);
                    break;
                }
            }
        } else if ((apiId & 0x300) == 0x300) {  // Parameter
#ifdef DEBUG_STATUS
            printREVFrame(rxFrame);
#endif
            Serial.print("Got parameter ");
            Serial.print(apiId & 0xFF, HEX);
            Serial.print(" for: ");
            for (int i = 0; i < MOTOR_AMOUNT; i++) {
                if (deviceId == motorList[i]->getID()) {
                    Serial.print(i);
                }
            }
            Serial.print(" (type ");
            Serial.print(rxFrame.data[4]);
            Serial.print("): ");
            //  uint32_t
            if (rxFrame.data[4] == static_cast<uint8_t>(sparkMax_ParameterType::kUint32)) {
                uint32_t val = (rxFrame.data[3] << 24) | (rxFrame.data[2] << 16) | (rxFrame.data[1] << 8) | rxFrame.data[0];
                Serial.print(val);
            //  int32_t  - Not sure if this one is actually right, copilot wrote it
            } else if (rxFrame.data[4] == static_cast<uint8_t>(sparkMax_ParameterType::kInt32)) {
                int32_t val = (rxFrame.data[3] << 24) | (rxFrame.data[2] << 16) | (rxFrame.data[1] << 8) | rxFrame.data[0];
                Serial.print(val);
            // float
            } else if (rxFrame.data[4] == static_cast<uint8_t>(sparkMax_ParameterType::kFloat32)) {
                uint32_t val = (rxFrame.data[3] << 24) | (rxFrame.data[2] << 16) | (rxFrame.data[1] << 8) | rxFrame.data[0];
                Serial.print(*reinterpret_cast<float*>(&val));
            // bool
            } else if (rxFrame.data[4] == static_cast<uint8_t>(sparkMax_ParameterType::kBool)) {
                Serial.print(rxFrame.data[0] ? "True" : "False");
            }
            // Error check
            if (rxFrame.data[5] != static_cast<uint8_t>(sparkMax_paramStatus::kOK)) {
                Serial.print(" - Error: ");
                switch (static_cast<sparkMax_paramStatus>(rxFrame.data[5])) {
                case sparkMax_paramStatus::kInvalidID:
                    Serial.print("Invalid ID");
                    break;
                
                case sparkMax_paramStatus::kMismatchType:
                    Serial.print("Mismatched Type");
                    break;
                
                case sparkMax_paramStatus::kAccessMode:
                    Serial.print("Access Mode");
                    break;
                
                case sparkMax_paramStatus::kInvalid:
                    Serial.print("Invalid");
                    break;
                
                case sparkMax_paramStatus::kNotImplementedDeprecated:
                    Serial.print("Deprecated or Not Implemented");
                    break;
                
                default:
                    Serial.print("Unknown");
                    break;
                }
            }
            Serial.println();
        }

#if defined(DEBUG_STATUS)
        // Log message if it seems interesting
        if (apiId == 0x99 || apiId == 0x60 || apiId == 0x61 || apiId == 0x62 || apiId == 0x63 || apiId == 0x64) {
            printREVFrame(rxFrame);
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
    if (COMMS_UART.available()) {
        String command = COMMS_UART.readStringUntil('\n');
        command.trim();

        COMMS_UART.println(command);

        static String prevCommand;

        std::vector<String> args = {};
        parseInput(command, args);

        //--------//
        //  Misc  //
        //--------//
        /**/ if (args[0] == "ping")
        {
#ifndef DEBUG
            COMMS_UART.println("pong");
#else
            Serial.println("pong");
            COMMS_UART.println("pong");
#endif
        } 
        else if (args[0] == "time") 
        {
            COMMS_UART.println(millis());
        }

        //-----------//
        //  Sensors  //
        //-----------//

        //----------//
        //  Motors  //
        //----------//
        else if (args[0] == "ctrl")
        {   
            lastCtrlCmd = millis();
            if (command != prevCommand)
            {
                prevCommand = command;

                setAxisSpeeds(args[1].toFloat(), args[2].toFloat(), args[3].toFloat());
                COMMS_UART.println("Motors Recieved Ctrl Command");
            }
        }
        else if (args[0] == "safetyOff")
        {
            safetyOn = false;
        }
        else if (args[0] == "Stop")
        {
            Stop();
        }
        else if (args[0] == "stop") // Stop a specific joint
        {
            motorList[args[1].toInt()-1]->stop();
        }

        else if (args[0] == "brake") 
        {
            if (args[1] == "on") 
            {
                Brake(true);
#ifdef DEBUG
                Serial.println("Setting brakemode on.");
#endif
            }

            else if (args[1] == "off")
            {
                Brake(false);
#ifdef DEBUG
                Serial.println("Setting brakemode off.");
#endif
            }
        }
#if defined(DEBUG2) && !defined(OLD_ASTRACAN_ENABLE)
        else if (args[0] == "id") {
            CAN_identifySparkMax(2);
        }
        else if (args[0] == "speed" && checkArgs(args, 1)) {
            CAN_sendVelocity(MOTOR_ID_BL, args[1].toFloat());
        }
        else if (args[0] == "newduty") {
            Serial.print("Setting duty cycle ");
            Serial.println(args[1].toFloat());
            CAN_sendDutyCycle(1, args[1].toFloat());
            CAN_sendDutyCycle(2, args[1].toFloat());
            CAN_sendDutyCycle(3, args[1].toFloat());
            CAN_sendDutyCycle(4, args[1].toFloat());
        }
        else if (args[0] == "stop") {
            Serial.println("Stopping all motors");
            for (int i = 0; i < 4; i++)
            {
                CAN_sendDutyCycle(i, 0);
                Stop();
            }
        }
        else if (args[0] == "turnby") {
            Motor2.turnByDeg(args[1].toFloat());
        }
        else if (args[0] == "forward") {
            driveMeters(args[1].toFloat());
        }
#endif
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

void safety_timeout()
{
    
    if (safetyOn && (millis() - lastCtrlCmd > 2000))  // if no control commands are received for 2 seconds
    {
        lastCtrlCmd = millis();

        COMMS_UART.println("No Control, Safety Timeout");
        Stop();
    }
}

void setAxisSpeeds(float A1Speed, float A2Speed, float A3Speed)
{
    COMMS_UART.println("Setting Motor Speeds");
    motorList[0]->setDuty(A1Speed);
    motorList[1]->setDuty(A2Speed);
    motorList[2]->setDuty(A3Speed);
}

void motorFeedback()
{
    for (int i = 1; i < 4; i++)
    { 
        String motor_feedback = 39+","+i+',';
        motor_feedback += motorList[i]->status1.busVoltage*10+',';
        motor_feedback += motorList[i]->status1.outputCurrent*10+',';
        motor_feedback += motorList[i]->status1.motorTemperature*10;
        COMMS_UART.println(motor_feedback);
    }
}

// Bypasses the acceleration to make the rover stop
// Should only be used for autonomy, but it could probably be used elsewhere
void Stop()
{
    for (int i = 0; i < MOTOR_AMOUNT; i++) {
        motorList[i]->stop();
    }
}

void updateMotorStatus()
{
    for (int i = 1; i < 4; i++) {
        String dataOut = "";
        dataOut += String(39)+",";
        dataOut += String(i)+","; 
        dataOut += String(motorList[i]->status1.motorTemperature*10)+",";
        dataOut += String(static_cast<int>(motorList[i]->status1.busVoltage*10))+",";
        dataOut += String(static_cast<int>(motorList[i]->status1.outputCurrent*10));
        COMMS_UART.println(dataOut);
    }
}

/*

// Bypasses the acceleration to make the rover turn clockwise
// Should only be used for autonomy
void turnCW()
{
    for (int i = 0; i < 4; i++)
        motorList[i]->sendDuty(0.6);
}

// Bypasses the acceleration to make the rover turn counterclockwise
// Should only be used for autonomy
void turnCCW()
{
    for (int i = 0; i < 4; i++)
        motorList[i]->sendDuty(-0.6);
}

// Bypasses the acceleration to make the rover stop
// Should only be used for autonomy, but it could probably be used elsewhere
void Stop()
{
    for (int i = 0; i < 4; i++) {
        motorList[i]->stop();
    }
}

*/
// Enables or disables brake mode for all motors
void Brake(bool enable) {
    for (int i = 0; i < MOTOR_AMOUNT; i++)
        motorList[i]->setBrake(enable);
}
/*

// Tells the rover to go forwards
// Does not bypass acceleration
// Autonomy
void goForwards(float speed)
{
    for (int i = 0; i < 4; i++ )
        motorList[i]->setDuty(speed);
}

// Tells the rover to go backwards
// Does not bypass acceleration
// Autonomy
void goBackwards(float speed)
{
    float temp = (-1) * speed;
    for (int i = 0; i < 4; i++ )
        motorList[i]->setDuty(temp);
}

void driveMeters(float meters) {
    const float degrees = (meters / WHEEL_CIRCUMFERENCE) * 360.0;

    // Left motors
    Motor1.turnByDeg(degrees);
    Motor2.turnByDeg(degrees);
    // Right motors
    Motor3.turnByDeg(-1 * degrees);
    Motor4.turnByDeg(-1 * degrees);
}

float getDriveSpeed() {
    float sum;
    for (int i = 0; i < MOTOR_AMOUNT; i++) {
        sum += abs(motorList[i]->status1.sensorVelocity);
    }
    const float avgSpeed = sum / 4;  // RPM
    const float gearBox = 64;  // 64:1 for testbed
    return (avgSpeed / gearBox) * WHEEL_CIRCUMFERENCE / 60;  // meters per second
}

*/
