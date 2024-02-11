// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <utility/imumaths.h>
// Our own resources
#include "AstraMotors.h"
//#include "AstraCAN.h"
//#include "AstraSensors.h"
#include "TeensyThreads.h"
#include "AstraSubroutines.h"


using namespace std;

#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

//Setting up for CAN0 line
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

//AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Axis1(2, 1, false, 50, 0.50F);//FL
AstraMotors Axis2(4, 1, false, 50, 0.50F);//BL
AstraMotors Axis3(1, 1, false, 50, 0.50F);//FR
//AstraMotors Axis3(3, 1, false, 50, 0.50F);//BR

AstraMotors motorList[3] = {Axis1, Axis2, Axis3};//Left motors first, Right motors Second
AccelStepper Axis0(AccelStepper::FULL2WIRE,2,3,4,5); // change pin locations

unsigned long lastAccel;
unsigned long lastDuty;
unsigned long lastHB;

unsigned long lastIdentify;

void loopHeartbeats(){
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

    while(1){
      sendHeartbeat(Can0, 1);
      threads.delay(30);
      sendHeartbeat(Can0, 2);
      threads.delay(30);
      sendHeartbeat(Can0, 3);
      threads.delay(30);
      sendHeartbeat(Can0, 4);
      threads.delay(30);
      threads.yield();
    }

}


void setup() {

  //-----------------//
  // Initialize Pins //
  //-----------------//
  
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    digitalWrite(LED_PIN, HIGH);

    delay(5000);
    digitalWrite(LED_PIN, LOW);

    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();

  //--------------------//
  // Initialize Sensors //
  //--------------------//
 
  //Start heartbeat thread
  //TEMPORARY FIX, until we get a dedicated microcontroller for heartbeat propogation
  threads.addThread(loopHeartbeats);

  
}



//------------//
// Begin Loop //
//------------//

void splitString(String comm, String prevComm, string tok, size_t pos, string scomm, string delim){
    scomm.erase(0, pos + delim.length()); 
    prevComm = comm; 
} 

void getToken(string token, size_t pos, string delimiter, string scommand){
    token = scommand.substr(0,pos); 
    pos = scommand.find(delimiter); 
}

void loop() {
  // Required To make the bmp not do stupid shit,
  // I am keeping it in this version so that I don't forget about it
  //Serial.println(bmp.temperature);
  

  // Accelerate the motors
  if(millis()-lastAccel >= 50){
    lastAccel = millis();
    for(int i = 0; i < 4; i++) {
      motorList[i].UpdateForAcceleration();
    }

    if(motorList[0].getControlMode() == 1)//send the correct duty cycle to the motors
    {
        for(int i = 0; i < 4; i++)
        {
          //sendHeartbeat(Can0, i+1);
          sendDutyCycle(Can0, motorList[i].getID(), motorList[i].getDuty());
          //Serial.println("Sending Duty Cycle");
        }
        /* Debug print duty cycles every 1 second
        if(millis()-lastDuty >= 1000)
        {
          lastDuty = millis();
          
          Serial.print("Duty1: ");
          Serial.print(motorList[0].getDuty());
          Serial.print("\tDuty2: ");
          Serial.println(motorList[1].getDuty());

          Serial.print("Duty3: ");
          Serial.print(motorList[2].getDuty());
          Serial.print("\tDuty4: ");
          Serial.println(motorList[3].getDuty());
        }
        */
    }else{
        //pass for RPM control mode
    }
 

  }


  //------------------//
  // Command Receiving //
  //------------------//
  //
  // Commands will be received as a comma separated value string
  // Ex: "ctrl,1,1,1,1" or "speedMultiplier,0.5" or "sendHealthPacket"
  // The program parses the string so that each piece of data can be used individually
  // For examples of parsing data you can use the link below
  // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c 

  
  

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // Command is equal to a line in the serial
    command.trim();                                 // I don't know why this is here, but it is important
    string delimiter = ",";                         // The key that decides where the command should be split
    size_t pos = 0;                                 // Standard parse variable
    string token;                             // The piece of string indicating a specific command
    string token2;                            // The current piece of the string being used.
    string scommand = command.c_str();         // Converts the Arduino String into a C++ string since they are different things
    pos = scommand.find(delimiter);
    token = scommand.substr(0, pos);
    String prevCommand;

    

    if (token == "ctrl") {                          // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
      if(command != prevCommand) {

          scommand.erase(0, pos + delimiter.length());
          prevCommand = command;

          for(int i = 0; i < 3; i+= 2){

            token = scommand.substr(0, pos);
            pos = scommand.find(delimiter);
            motorList[i].setDuty(stof(token));
            motorList[i+1].setDuty(stof(token));
            scommand.erase(0, pos + delimiter.length());
          }
        } 
    } else if (token == "arm") {                 // Is looking for a command -> "armMove,move,duty" where duty is >-1 and <1
        if (command != prevCommand) {
          splitString(command,prevCommand,token,pos,scommand,delimiter);

          if (token == "move") { // "arm,move,duty1,duty2,duty3,target" note: change to accept angles after testing arm
            for (int i = 1; i < 4; i++) {
              getToken(token,pos,delimiter,scommand);
              motorList[i].setDuty(stof(token)); 
            }
            Axis0.moveTo(stof(token));                 // set target location for stepper motor
            Axis0.run();                               // run stepper motor until target is met
          } else if (token == "axis") {                // axis 1-3 control with format "arm,axis,1,duty"

            scommand.erase(0, pos+delimiter.length()); // remove previous token
            getToken(token,pos,delimiter,scommand);    // get new token
            int index = stoi(token);                   // get index of selected axis
            scommand.erase(0, pos + delimiter.length()); // repeat above
            getToken(token,pos,delimiter,scommand);

            if (index != 0) {
              motorList[index].setDuty(stoi(token));   // set duty cycle of indicated axis
            }
              else if (index == 0) {                     // axis 0 control with format "arm,axis,0,target"
              Axis0.move(stoi(token));
              Axis0.run(); 
            }
          } else if  (token == "stop") {               // stop movement of all axes 
            for (int j = 1; j < 4; j++) {
              motorList[j].setDuty(0); 
            }
            Axis0.stop(); 
          } else if  (token == "stepper") { 
            scommand.erase(0, pos + delimiter.length()); 
            getToken(token,pos,delimiter,scommand); 

            if (token == "speed") {
              scommand.erase(0, pos+delimiter.length());
              getToken(token,pos,delimiter,scommand); 

              Serial.print("Previous: ");
              Serial.print(Axis0.speed());
              Serial.print(" Current: "); 
              Axis0.setSpeed(stof(token));
              Axis0.setMaxSpeed(stof(token)); 
              Serial.println(Axis0.speed()); 

            } else if (token == "accel"){
              scommand.erase(0, pos+delimiter.length());
              getToken(token,pos,delimiter,scommand); 

              Axis0.setAcceleration(stof(token));

            } else if (token == "calibrate"){
              scommand.erase(0, pos+delimiter.length());
              getToken(token,pos,delimiter,scommand);

              float temp = Axis0.speed(); 
              Serial.print(Axis0.currentPosition()); 
              Serial.println(" now set as 0");
              Axis0.setCurrentPosition(Axis0.currentPosition());
              Axis0.setSpeed(temp); 
            } else if (token == "fullstop") {
              Axis0.stop(); 
              Axis0.disableOutputs(); 
            } else if (token == "restart") {
              Axis0.enableOutputs(); 
            }
          }
        }
    }else if (token == "endEffect"){
      if (command != prevCommand) {
        // what am I interacting with to control the end effector? 
      }
    }else if (token == "data") {                          
      if(command != prevCommand) {
        scommand.erase(0, pos + delimiter.length());
        prevCommand = command;
        pos = scommand.find(delimiter);
        token = scommand.substr(0, pos);
      } 
    }
  }
}
