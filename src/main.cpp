/**
 * @file main.cpp
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Digit PCB's Pico code. Controls end effector.
 * 
 */

//----------//
// Includes //
//----------//

#include <Arduino.h>

#include <vector>

#include "AstraMisc.h"
#include "project/DIGIT.h"

using namespace std;


//-------------//
// Global vars //
//-------------//

#define BLINK
// #define DEBUG


// millis value of last control command received
unsigned long lastCtrlCmd = millis();
// Whether the last servo control was due to control command (1) or timeout (0)
bool EFcontrolReal;

// millis value of last LED blink
unsigned long lastBlink = 0;
bool ledState = true;


//------------//
// Prototypes //
//------------//


//-------------//
// Begin Setup //
//-------------//

void setup() {
    //-----------------//
    // Initialize Pins //
    //-----------------//

    // Serial and onboard LED
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(SERIAL_BAUD);
    COMMS_UART.begin(COMMS_UART_BAUD);

    // Show with LED that pico is starting
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);

    // Laser
    pinMode(PIN_LASER, OUTPUT);
    digitalWrite(PIN_LASER, LOW);
}


//------------//
// Begin Loop //
//------------//
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
    // Blink LED
#ifdef BLINK
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
#endif

    // Motor control timeout
    if (millis() - lastCtrlCmd > 1000) {  // temporarily set to 1 second
        lastCtrlCmd = millis();
        EFcontrolReal = 0;
#ifdef DEBUG
        Serial.println("Stopping servo");
#endif
    }


    //-------------------//
    // Command Receiving //
    //-------------------//

    if (COMMS_UART.available()) {
        String command = COMMS_UART.readStringUntil('\n');
        command.trim();
        command.toLowerCase();
#ifdef DEBUG
        Serial.print("[IN]: ");
        Serial.print(command);
        Serial.print('\n');
#endif

        std::vector<String> args = {};
        parseInput(command, args, ',');

        // Comes from ROS -> socket raspi ->UART-> socket teensy 4.1 ->UART-> Digit
        if (command == "digit") {
            // Remove first argument, which is "digit" to tell socket teensy to redirect to digit
            args.erase(args.begin());
            // Our command is not "digit", but what comes after it
            command = args[0];
            command.toLowerCase();
        }

        if (args[0] == "ctrl") {  // Is looking for a command that looks like
                                  // "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
#ifdef DEBUG
            Serial.println("controling");
#endif
            lastCtrlCmd = millis();
            EFcontrolReal = 1;
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
                digitalWrite(PIN_LASER, LOW);
            } else if (args[1] == "1") {
                digitalWrite(PIN_LASER, HIGH);
            }

        } else if (args[0] == "ping") {
            Serial.println("pong");
            COMMS_UART.println("pong");
        } else if (args[0] == "time") {
            Serial.println(millis());
        }
    }

    // Take input from USB for debugging
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        std::vector<String> args = {};
        parseInput(command, args, ',');
#ifdef DEBUG
        Serial.print("GOT from Serial: ");
        Serial.println(command);
        // COMMS_UART.print("faeriesht,");
        // COMMS_UART.print("hereiscommand,");
        // COMMS_UART.println(command);
#endif
    }
}
