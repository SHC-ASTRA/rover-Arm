// Includes
#include <Arduino.h>
#include <Servo.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "AstraMisc.h"
#include "project/DIGIT.h"

using namespace std;

Servo efMotor;

#define PIN_LASER 8  // GPIO 8  // moved to rover-Embedded-Lib in 0.6.0

#define BLINK
// #define DEBUG


unsigned long clockTimer = millis();
bool EFcontrolReal;

unsigned long lastBlink = 0;
bool ledState = true;


void setup() {
    //-----------------//
    // Initialize Pins //
    //-----------------//

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(SERIAL_BAUD);
    COMMS_UART.begin(COMMS_UART_BAUD);
    digitalWrite(LED_BUILTIN, HIGH);

    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);

    // pinMode(20, INPUT_PULLUP); //Needed for IMU to work on PCB <-this line is from core rover,
    // but leaving it here
    pinMode(PIN_LASER, OUTPUT);  // GPIO 8
    digitalWrite(PIN_LASER, LOW);

    efMotor.attach(PIN_EF_MOTOR);  // GPIO 19, Physically pin 25
}

void EFcontrol(float speed, bool &moveT_F);



//------------//
// Begin Loop //
//------------//

void loop() {
#ifdef BLINK
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
#endif


    if (millis() - clockTimer > 1000) {  // temporarily set to 1 second
        clockTimer = millis();
        EFcontrolReal = 0;
        efMotor.writeMicroseconds(1500);
        // COMMS_UART.print("PING\n");
        // Serial.println("PING\n");
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
#ifdef DEBUG
        Serial.print("[IN]: ");
        Serial.print(command);
        Serial.print('\n');
#endif

        std::vector<String> args = {};
        parseInput(command, args, ',');

        if (args[0] == "digit") {  // Is looking for a command that looks like
                                   // "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1

            if (args[1] == "ctrl") {
                clockTimer = millis();
                EFcontrolReal = 1;
                if (args[2] == "1")  // close
                {
                    EFcontrol(0, EFcontrolReal);  // 0(close)-2500(open) with 1500 as stop
                } else if (args[2] == "-1")       // open
                {
                    EFcontrol(2500, EFcontrolReal);  // 0(close)-2500(open) with 1500 as stop
                } else if (args[2] == "0")           // stop
                {
                    EFcontrol(1500, EFcontrolReal);  // 0(close)-2500(open) with 1500 as stop
                }

#ifdef DEBUG
                Serial.println("controling");
#endif
            } else if (args[1] == "laser") {  //
                if (args[2] == "0") {
                    digitalWrite(PIN_LASER, LOW);
                } else if (args[2] == "1") {
                    digitalWrite(PIN_LASER, HIGH);
                }
            }

        } else if (args[0] == "ping") {
            Serial.println("pong");
            COMMS_UART.println("pong");
        } else if (args[0] == "time") {
            Serial.println(millis());
        }
    }

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


void EFcontrol(float speed, bool &moveT_F) {
    if (moveT_F) {
        efMotor.writeMicroseconds(speed);
    }
}
