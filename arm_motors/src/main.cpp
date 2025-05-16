/**
 * @file main.cpp
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
#include "AstraREVCAN.h"
#include "AstraMotors.h"
#include "ArmMotorMCU.h"


//------------//
//  Settings  //
//------------//

#if !defined(DEBUG)
#    ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
#        define COMMS_UART Serial1  // To/from Main MCU
#    else
#        define COMMS_UART Serial2  // To/from Main MCU
#    endif
#else
#    define COMMS_UART Serial  // To/from USB for debugging
#endif

#define MOTOR_ID_A0 4  // To be set
#define MOTOR_ID_A1 1
#define MOTOR_ID_A2 2
#define MOTOR_ID_A3 3

#define MOTOR_AMOUNT 4


//---------------------//
//  Component classes  //
//---------------------//

// AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors MotorAxis0(MOTOR_ID_A0, sparkMax_ctrlType::kDutyCycle, true);
AstraMotors MotorAxis1(MOTOR_ID_A1, sparkMax_ctrlType::kDutyCycle, true);
AstraMotors MotorAxis2(MOTOR_ID_A2, sparkMax_ctrlType::kDutyCycle, true);
AstraMotors MotorAxis3(MOTOR_ID_A3, sparkMax_ctrlType::kDutyCycle, true);

AstraMotors* motorList[] = {&MotorAxis0, &MotorAxis1, &MotorAxis2, &MotorAxis3};


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

unsigned long lastHB = 0;
int heartBeatNum = 1;

unsigned long lastCtrlCmd = 0;
unsigned long lastMotorStatus = 0;
bool safetyOn = true;

uint32_t lastAccel = 0;

#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2

hw_timer_t *Timer0_Cfg = NULL, *Timer1_Cfg = NULL;

void IRAM_ATTR Timer0_ISR() {
    CAN_sendHeartbeat(heartBeatNum);
    heartBeatNum++;
    if (heartBeatNum > 4)
    {
        heartBeatNum = 1;
    }
}

#else

void loop2(void* pvParameters) {
    while (true) {
        CAN_sendHeartbeat(heartBeatNum);
        heartBeatNum++;
        if (heartBeatNum > 4)
        {
            heartBeatNum = 1;
        }
        delay(5);
    }
}

#endif


//--------------//
//  Prototypes  //
//--------------//

void Brake(bool enable);
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

    // Setup CAN
    if (ESP32Can.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX)) 
    {
        COMMS_UART.println("CAN bus started!");
    } 
    else 
    {
        COMMS_UART.println("CAN bus failed!");
    }

    //-----------//
    //  Sensors  //
    //-----------//

    //--------------------//
    //  Misc. Components  //
    //--------------------//

    #ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2

    Timer0_Cfg = timerBegin(0, 80, true);
    timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    timerAlarmWrite(Timer0_Cfg, 5000, true);
    timerAlarmEnable(Timer0_Cfg);

    #else

    xTaskCreatePinnedToCore (
        loop2,     // Function to implement the task
        "loop2",   // Name of the task
        1000,      // Stack size in bytes
        NULL,      // Task input parameter
        0,         // Priority of the task
        NULL,      // Task handle.
        0          // Core where the task should run
    );

    #endif
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

    // // Accelerate motors; update the speed for all motors
    // if (millis() - lastAccel >= 100) 
    // {
    //     lastAccel = millis();
    //     for (int i = 0; i < MOTOR_AMOUNT; i++) 
    //     {
    //         motorList[i]->accelerate();
    //     }
    // }

    // Heartbeat moved to interrupt timer

    // Safety timeout
    if (safetyOn && millis() - lastCtrlCmd > 2000)
    {
        lastCtrlCmd = millis();

        COMMS_UART.println("No Control, Safety Timeout");
        Stop();
    }

    // Motor status
    if (millis() - lastMotorStatus > 500) { 
        lastMotorStatus = millis();

        for (int i = 0; i < MOTOR_AMOUNT; i++) {
            if (millis() - motorList[i]->status1.timestamp > 500)  // Don't send outdated data
                continue;
            COMMS_UART.printf("motorstatus,%d,%d,%d,%d\n", motorList[i]->getID(), int(motorList[i]->status1.motorTemperature * 10),
                int(motorList[i]->status1.busVoltage * 10), int(motorList[i]->status1.outputCurrent * 10));
        }
    }


    //-------------//
    //  CAN Input  //
    //-------------//

    static CanFrame rxFrame;
    if (ESP32Can.readFrame(rxFrame, 1)) {
        uint8_t deviceId = rxFrame.identifier & 0x3F;  // [5:0]
        uint32_t apiId = (rxFrame.identifier >> 6) & 0x3FF;  // [15:6]

#if defined(DEBUG_STATUS)
        // Log message if it seems interesting
        if (apiId == 0x99 || (apiId & 0x60) == 0x60 || (apiId & 0x300) == 0x300) {
            printREVFrame(rxFrame);
        }
#endif

        if ((apiId & 0x60) == 0x60) {  // Periodic status
            for (int i = 0; i < MOTOR_AMOUNT; i++) {
                if (deviceId == motorList[i]->getID()) {
                    motorList[i]->parseStatus(apiId, rxFrame.data);
                    break;
                }
            }
        }
        else if ((apiId & 0x300) == 0x300) {  // Parameter
            printREVParameter(rxFrame);
#ifdef DEBUG
            Serial.print("From frame: ");
            printREVFrame(rxFrame);
#endif
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
    if (COMMS_UART.available()) {
        String input = COMMS_UART.readStringUntil('\n');
        input.trim();
        std::vector<String> args = {};
        parseInput(input, args);
        args[0].toLowerCase();

#ifdef ARDUINO_ADAFRUIT_FEATHER_ESP32_V2
        Serial1.println(input);
#else
        Serial2.println(input);
#endif
        Serial.println(input);

        //--------//
        //  Misc  //
        //--------//

        if (args[0] == "ping")
        {
            COMMS_UART.println("pong");
            Serial.println("pong");
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
            motorList[0]->sendDuty(args[1].toFloat());
            motorList[1]->sendDuty(args[2].toFloat());
            motorList[2]->sendDuty(args[3].toFloat());
            motorList[3]->sendDuty(args[4].toFloat());
        }

        else if (args[0] == "sendvelocity") {
            lastCtrlCmd = millis();
            motorList[0]->sendSpeed(args[1].toFloat());
            motorList[1]->sendSpeed(args[2].toFloat());
            motorList[2]->sendSpeed(args[3].toFloat());
            motorList[3]->sendSpeed(args[4].toFloat());
        }

        else if (args[0] == "safetyoff")
        {
            safetyOn = false;
        }

        else if (args[0] == "stop")
        {
            if (checkArgs(args, 1) && args[1].toInt() > 0 && args[1].toInt() <= MOTOR_AMOUNT)
            {
                motorList[args[1].toInt()-1]->stop();  // Stop a specific joint
            }
            else
            {
                Stop();
            }
        }

        else if (args[0] == "brake") 
        {
            if (args[1] == "on") 
            {
                Brake(true);
            }

            else if (args[1] == "off")
            {
                Brake(false);
            }
        }

        else if (args[0] == "id") {
            CAN_identifySparkMax(args[1].toInt());
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

// Enables or disables brake mode for all motors
void Brake(bool enable) {
    for (int i = 0; i < MOTOR_AMOUNT; i++)
        motorList[i]->setBrake(enable);
}

// Bypasses the acceleration to make the rover stop immediately
void Stop()
{
    for (int i = 0; i < MOTOR_AMOUNT; i++) {
        motorList[i]->stop();
    }
}
