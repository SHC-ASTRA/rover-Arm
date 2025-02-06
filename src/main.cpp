/**
 * @file Template.cpp
 * @author your name (you@domain.com)
 * @brief description
 *
 */

//------------//
//  Includes  //
//------------//

#include <Arduino.h>
#include "AstraMisc.h"
#include "project/DIGIT.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK


//---------------------//
//  Component classes  //
//---------------------//


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

unsigned long lastCtrlCmd = millis();


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

    pinMode(LASER_NMOS, OUTPUT);
    digitalWrite(LASER_NMOS, LOW);


    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);
    Serial1.begin(COMMS_UART_BAUD);


    //-----------//
    //  Sensors  //
    //-----------//


    //--------------------//
    //  Misc. Components  //
    //--------------------//

    // LSS will go here
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

    // Motor control timeout
    if (millis() - lastCtrlCmd > 2000) {
        lastCtrlCmd = millis();
        // Stop LSS
        // Stop EF motor
        // Stop lin ac
#ifdef DEBUG
        Serial.println("Control timeout");
#endif
    }


    //-------------//
    //  CAN Input  //
    //-------------//


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

        //-----------//
        //  Sensors  //
        //-----------//

        //----------//
        //  Motors  //
        //----------//

        else if (args[0] == "ctrl") {  // Is looking for a command that looks like
                                  // "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
#ifdef DEBUG
            Serial.println("controling");
#endif
            lastCtrlCmd = millis();
            if (args[1] == "1")  // close
            {
                // EFcontrol(0, EFcontrolReal);  // 0(close)-2500(open) with 1500 as stop
            } else if (args[1] == "-1")       // open
            {
                // EFcontrol(2500, EFcontrolReal);  // 0(close)-2500(open) with 1500 as stop
            } else if (args[1] == "0")           // stop
            {
                // EFcontrol(1500, EFcontrolReal);  // 0(close)-2500(open) with 1500 as stop
            }

        } else if (args[0] == "laser") {
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
