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

    // Laser
    pinMode(LASER_NMOS, OUTPUT);
    digitalWrite(LASER_NMOS, LOW);

    // Linear actuator
    pinMode(LINAC_RIN, OUTPUT);
    pinMode(LINAC_FIN, OUTPUT);
    digitalWrite(LINAC_RIN, LOW);
    digitalWrite(LINAC_FIN, LOW);


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
        digitalWrite(LINAC_RIN, LOW);
        digitalWrite(LINAC_FIN, LOW);
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
