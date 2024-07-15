// Includes
#include <Arduino.h>
#include <cmath>
#include <cstdlib>
#include <vector>

#include "DIGIT.h"

// Included with rover-Embedded-Lib
/*
#include <Servo.h>
#include <iostream>
#include <string>
*/

using namespace std;

Servo myservo;

#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)
#define LASER_PIN 8 //GPIO 8


unsigned long clockTimer = millis();
bool EFcontrolReal;


void setup() {

  //-----------------//
  // Initialize Pins //
  //-----------------//
  
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    Serial1.begin(115200);
    digitalWrite(LED_PIN, HIGH);

    delay(2000);
    digitalWrite(LED_PIN, LOW);

    //pinMode(20, INPUT_PULLUP); //Needed for IMU to work on PCB <-this line is from core rover, but leaving it here
    pinMode(LASER_PIN, OUTPUT); //GPIO 8
    digitalWrite(LASER_PIN, LOW);

    myservo.attach(19); //GPIO 19, Physically pin 25
  
}

void EFcontrol(float speed, bool &moveT_F);
void parseInput(const String input, std::vector<String>& args, char delim);



//------------//
// Begin Loop //
//------------//

void loop() {


  if((millis()-clockTimer)>1000){//temporarily set to 1 second
    clockTimer = millis();
    EFcontrolReal = 0;
    myservo.writeMicroseconds(1500);
    // Serial1.print("PING\n");
    // Serial.println("PING\n");
    //Serial.println("Stopping servo");
  }


  //------------------//
  // Command Receiving //
  //------------------//
  //

  if (Serial1.available()) {
    String command = Serial1.readStringUntil('\n');  
    command.trim();
    //Serial.print("[IN]:" + command + "\n");
    
                      
    std::vector<String> args = {};
    parseInput(command, args, ',');

    if (args[0] == "digit") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        
      if(args[1] == "ctrl")
      {
        clockTimer = millis();
        EFcontrolReal = 1;
        if(args[2] == "1")//close
        {
            EFcontrol(0, EFcontrolReal); // 0(close)-2500(open) with 1500 as stop
        }else if(args[2] == "-1")//open
        {
            EFcontrol(2500, EFcontrolReal); // 0(close)-2500(open) with 1500 as stop
        }else if(args[2] == "0")//stop
        {
            EFcontrol(1500, EFcontrolReal); // 0(close)-2500(open) with 1500 as stop
        }

        Serial.println("controling");
      }else if (args[1] == "laser") {        // 
        if(args[2] == "0")
        {
          digitalWrite(LASER_PIN, LOW);
        }else if(args[2] == "1")
        {
          digitalWrite(LASER_PIN, HIGH);
        }
      }

    }else if (args[0] == "ping") {
      Serial.println("pong");
      Serial1.println("pong");
    } else if (args[0] == "time") {
      Serial.println(millis());
    }


  }

}




void EFcontrol(float speed, bool &moveT_F){
  if(moveT_F){
    myservo.writeMicroseconds(speed);
  }
}


// Parse `input` into `args` separated by `delim`
// Ex: "ctrl,led,on" => {ctrl,led,on}
// Equivalent to Python's `.split()`
void parseInput(const String input, std::vector<String>& args, char delim) {
    //Modified from https://forum.arduino.cc/t/how-to-split-a-string-with-space-and-store-the-items-in-array/888813/9

    // Index of previously found delim
    int lastIndex = -1;
    // Index of currently found delim
    int index = -1;
    // because lastIndex=index, lastIndex starts at -1, so with lastIndex+1, first search begins at 0

    // if empty input for some reason, don't do anything
    if(input.length() == 0)
        return;

    unsigned count = 0;
    while (count++, count < 200 /*arbitrary limit on number of delims because while(true) is scary*/) {
        lastIndex = index;
        // using lastIndex+1 instead of input = input.substring to reduce memory impact
        index = input.indexOf(delim, lastIndex+1);
        if (index == -1) { // No instance of delim found in input
            // If no delims are found at all, then lastIndex+1 == 0, so whole string is passed.
            // Otherwise, only the last part of input is passed because of lastIndex+1.
            args.push_back(input.substring(lastIndex+1));
            // Exit the loop when there are no more delims
            break;
        } else { // delim found
            // If this is the first delim, lastIndex+1 == 0, so starts from beginning
            // Otherwise, starts from last found delim with lastIndex+1
            args.push_back(input.substring(lastIndex+1, index));
        }
    }

    // output is via vector<String>& args
}