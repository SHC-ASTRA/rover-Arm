/**
 * @file Main.cpp
 * @author Jack Schumacher (js0342@uah.edu)
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Arm Embedded
 *
 */

//------------//
//  Includes  //
//------------//

#include <Arduino.h>
#include <cmath>
#include <utility/imumaths.h>

// Our own resources
#if defined(TESTBED)
#include "project/TESTBED.h"
#else
#include "project/CORE.h"
#endif
#include "AstraMisc.h"
#include "AstraVicCAN.h"
#include "AstraSensors.h"

//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

AstraCAN Can0;

//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

unsigned long clockTimer = 0;
unsigned long lastFeedback = 0;
unsigned long lastCtrlCmd = 0;

//--------------//
//  Prototypes  //
//--------------//

void safety_timeout();

//--------//
//  Misc  //
//--------//

String feedback;

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
void setup()
{
    //--------//
    //  Pins  //
    //--------//

    pinMode(LED_BUILTIN, OUTPUT);

    //-----------//
    //  MCU LED  //
    //-----------//

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);
    COMMS_UART.begin(COMMS_UART_BAUD);

    if (Can0.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX))
        Serial.println("CAN bus started!");
    else
        Serial.println("CAN bus failed!");
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
void loop()
{
    //----------//
    //  Timers  //
    //----------//
#ifdef BLINK
    if (millis() - lastBlink > 1000)
    {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
#endif

    if ((millis() - lastFeedback) >= 2000)
    {
        //    To add later
    }

    // Safety timeout if no ctrl command for 2 seconds
    safety_timeout();

    //-------------//
    //  CAN Input  //
    //-------------//

    static CanFrame rxFrame;
    if (Can0.readFrame(rxFrame, 100))
    {
        Serial.printf("Received frame: %03X  \r\n", rxFrame.identifier);
        // Vehicle CAN code will go here
    }

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
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');

        input.trim();                  // Remove preceding and trailing whitespace
        std::vector<String> args = {}; // Initialize empty vector to hold separated arguments
        parseInput(input, args, ',');  // Separate `input` by commas and place into args vector
        args[0].toLowerCase();         // Make command case-insensitive
        String command = args[0];      // To make processing code more readable

        String prevCommand;

        //--------//
        //  Misc  //
        //--------//
        /**/ if (command == "ping")
        {
            Serial.println("pong");
        }

        else if (command == "time")
        {
            Serial.println(millis());
        }

        else if (command == "led")
        {
            if (args[1] == "on")
                digitalWrite(LED_BUILTIN, HIGH);
            else if (args[1] == "off")
                digitalWrite(LED_BUILTIN, LOW);
            else if (args[1] == "toggle")
            {
                ledState = !ledState;
                digitalWrite(LED_BUILTIN, ledState);
            }
        }

        //-----------//
        //  Sensors  //
        //-----------//

        else if (args[0] == "data") // Send data out
        {
            //   TBD
        }

        //------------//
        //  Physical  //
        //------------//

        // Relay data from the motor controller back over USB
        if (COMMS_UART.available())
        {
            String input = COMMS_UART.readStringUntil('\n');
            input.trim();
            Serial.println(input);
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
        if (millis() - lastCtrlCmd > 2000) // if no control commands are received for 2 seconds
        {
            lastCtrlCmd = millis(); // just update the var so this only runs every 2 seconds.

            Serial1.println("ctrl,0,0");
            Serial.println("No Control / Safety Timeout");
        }
    }
