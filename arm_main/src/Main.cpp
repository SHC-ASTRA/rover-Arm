// Includes
#include <Arduino.h>
#include <iostream>
#include <string>
#include <cmath>
#include <cstdlib>
#include <queue> 
#include <utility/imumaths.h>
#include <AS5047P.h> 
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

#define AS5047P_CHIP_SELECT_PORT 10 
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

#define SEALEVELPRESSURE_HPA (1013.25)

//Setting up for myCan line
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;

//Setting up for magnetic encoders 
 AS5047P encoder1(36, AS5047P_CUSTOM_SPI_BUS_SPEED); 
//  AS5047P encoder2(10, AS5047P_CUSTOM_SPI_BUS_SPEED);
// AS5047P encoder3(37, AS5047P_CUSTOM_SPI_BUS_SPEED);
 AS5047P encoders[3] = {encoder1}; 


//AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)
AstraMotors Axis1(1, 1, false, 50, 0.50F);//FL
AstraMotors Axis2(2, 1, false, 50, 0.50F);//BL
AstraMotors Axis3(3, 1, false, 50, 0.50F);//FR
//AstraMotors Axis3(3, 1, false, 50, 0.50F);//BR

AstraMotors motorList[3] = {Axis1, Axis2, Axis3};//Left motors first, Right motors Second
AccelStepper Axis0(AccelStepper::FULL2WIRE,2,3,4,5); // change pin locations

unsigned long lastAccel;
unsigned long lastDuty;
unsigned long lastHB;
unsigned long lastIdentify;
//unsigned long lastTime; 

const float forwardDuty = 0.1; 
const float backDuty = -0.1; 
const float ratio90 = (0.2/30); // at speed 0.2, takes 30 seconds to travel 90 degrees
const int limits[3] = {90,90,90}; 
const int motor_num = 3; // Number of motors in motorList
const int UP = 1; 
const int DOWN = 2; 
const int FRONT = 3; 
const int BACK = 4; 
const int COMMAND_LIMIT = 30; // Maximum number of tokens in command

char   allchars[COMMAND_LIMIT]; // Character buffer for collecting characters into string
char*  segments[COMMAND_LIMIT]; // Mediary layer for converting character buffer into string
queue<string> command_queue; 
string tokens[COMMAND_LIMIT];   // Tokens generated from command
int    char_index = 0;          // Index of allchars, used for determining position
bool   fixit = false;           // Variable for interpreting VSCode serial monitor (removes 1 character from start of array if true)
bool   moving = false;
bool   data_bool = false;   

// Function prototypes 
void  loopHeartbeats(); 
void  turnTime(int time, float duty, int index = 5); // turn motors for amount of time in ms
void  splitString(String comm, String prevComm, string tok, size_t pos, string scomm, string delim); // split string into tokens (deprecated)
void  getToken(string token, size_t pos, string delimiter, string scommand); // get individual token from command (deprecated)
void  moveStepper(float stickValue); // move stepper motor stickValue amount of steps
void  moveAxis(float stickValue, int index); // move individual axis with duty cycle stickValue and index 
int   getAngle(int index);  // get angle from magnetic encoders 
float convertDuty(float duty); // convert duty value to one acceptable by AstraMotors functions 
float convertDuty(int duty);   // convert duty value to one acceptable by AstraMotors functions
void  checkAxes(); // check for collision with another axis
void  charpump(char* tokes[COMMAND_LIMIT],string str); // convert character array 'tokes' to string
void  movebyAngle(int angle, int axis, float duty); 

size_t posFlip(size_t pos, string delimiter, string scommand);  // flip delimiter for determing pos (deprecated)
void   strdump(char* strings[COMMAND_LIMIT], string result[COMMAND_LIMIT]); // dump character array contents into string array
void   fixString(string& str , bool &fix); // fix incoming string when using VSCode serial monitor
//void   test(); // nothing lives and breathes to the fullest extent





void setup() {

  //-----------------//
  // Initialize Pins //
  //-----------------//
  
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    digitalWrite(LED_PIN, HIGH);

    delay(5000);
    digitalWrite(LED_PIN, LOW);

    myCan.begin(); // Initialize CAN bus settings
    myCan.setBaudRate(1000000);
    myCan.setMaxMB(16);
    myCan.enableFIFO();
    myCan.enableFIFOInterrupt();

  //--------------------//
  // Initialize Sensors //
  //--------------------//
 
  //sendHeartbeat(myCan,2); 

  //Start heartbeat thread
  //TEMPORARY FIX, until we get a dedicated microcontroller for heartbeat propogation
  threads.addThread(loopHeartbeats);
  //threads.addThread(test); 

   while (!encoder1.initSPI()) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(1000);
  }
  
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
  // Required To make the bmp not do stupid shit,
  // I am keeping it in this version so that I don't forget about it
  //Serial.println(bmp.temperature);
  
   //identifyDevice(myCan,1);
   //sendHeartbeat(myCan, 1);
   //Serial.println("sent"); 
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
          sendDutyCycle(myCan, motorList[i].getID(), motorList[i].getDuty()); // update motors with current duty cycle
        }
    } else{
        //pass for RPM control mode
    }
  }


  //------------------//
  // Command Receiving //
  //------------------//
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
  //
  // Commands will be received as a comma separated value string
  // Ex: "ctrl,1,1,1,1" or "speedMultiplier,0.5" or "sendHealthPacket"
  // The program parses the string so that each piece of data can be used individually
  // For examples of parsing data you can use the link below
  // https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c 

//identifyDevice(myCan,2);   
  
  //sendDutyCycle(myCan,motorList[0].getID(),0.3);

  if (Serial.available()) {
    //motorList[0].setDuty(0.3);  
     
    String command = ""; // Serial edition change
    //String command = Serial.readStringUntil('\n');  // Command is equal to a line in the serial
    command.trim();                                 // I don't know why this is here, but it is important
    string delimiter = ",";                         // The key that decides where the command should be split
    size_t pos = 0;                                 // Standard parse variable
    string token;                             // The piece of string indicating a specific command
    //string token2;                            // The current piece of the string being used.
    string scommand = "";         // Converts the Arduino String into a C++ string since they are different things
    
    pos = scommand.find(delimiter);
    token = scommand.substr(0, pos);
    String prevCommand;

    

   
   // SERIAL EDITS BEGIN HERE
    char acter = Serial.read(); 
    if (acter != '*') { // DELIMETER IS EXTREMELY IMPORTANT FOR COMMAND INTERPRETATION
        allchars[char_index] = acter; 
        char_index++;  
    }
    else {
      scommand = ""; 
      for (int j = 0; j < char_index; j++){
        if (allchars[j] != '\n' && allchars[j] != '\0') 
        {scommand += allchars[j];}
      }
      command = String(scommand.c_str()); 
      command_queue.push(scommand); 
      if (!command_queue.empty() && !moving) {
        scommand = command_queue.front(); 
        command_queue.pop(); 
      }
      
      fixString(scommand,fixit); // fix input command depending on serial connection used
      charpump(segments,scommand); // parse string into tokens
      strdump(segments,tokens); // convert tokens back into string
      Serial.println(String(token.c_str()));
      Serial.println("scommand: " + String(scommand.c_str()));
    // Major Serial Edits END HERE


      if (String(token.c_str()) == String("ctrl")) {    // Serial change    // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        if(command != prevCommand) {

            prevCommand = command;

            for(int i = 0; i < 3; i+= 2){
              float duty = convertDuty(stof(token)); 
              motorList[i].setDuty(duty);
              motorList[i+1].setDuty(duty);
            }
          } 
      } else if (tokens[0] == "arm") {   // Serial change               // Is looking for a command -> "armMove,move,duty" where duty is >-1 and <1
          if (command != prevCommand) {

            prevCommand = command; 
            if (tokens[1] == "move") { // "arm,move,duty1,duty2,duty3,target" note: change to accept angles after testing arm

              for (int i = 0; i < motor_num; i++) {
                motorList[i].setDuty(convertDuty(stof(tokens[i + 2]))); 
              }
              //Axis0.move(stof(token));                 // set target location for stepper motor
              //Axis0.run();                               // run stepper motor until target is met

            } else if (tokens[1] == "axis") {                // axis 1-3 control with format "arm,axis,1,duty"

              int index = stoi(tokens[2]); 
              float duty = stof(tokens[3]);                   // get index of selected axis

              if (index != 4) {
                motorList[index].setDuty(duty);   // set duty cycle of indicated axis
              }
                else if (index == 4) {                     // axis 0 control with format "arm,axis,0,target"
                //Axis0.move(stoi(token));
                //Axis0.run(); 
              }
              Serial.println("moving axis " + String(index) + " " + String(duty)); 

            } else if  (tokens[1] == "stop") {               // "arm,stop"

              for (int j = 0; j < 3; j++) {
                motorList[j].setDuty(convertDuty(0)); 
                motorList[j].setSpeed(0);
              }
              //Axis0.stop(); 
            } else if  (tokens[1] == "hey") { // "arm,hey"

              Serial.println("Hey"); 
            } else if  (tokens[1] == "time")  { // "arm,time,ms,axis"
                   
              int time = stoi(tokens[2]); // how long to run for
              int index = stoi(tokens[3]); // index of selected motor

              Serial.print("index: " + String(index));
              Serial.print(" time: " + String(time)); 

              turnTime(time,0.25F,index);
            } else if  (tokens[1] == "test") {

              Serial.println("testing begin:"); 
              // int angle = stoi(tokens[2]); 
              // int axis = stoi(tokens[3]); 
              // float duty = stof(tokens[4]);
              movebyAngle(10,2,0.1); 
              motorList[1].setDuty(0);    
              Serial.println("testing over"); 
            } else if (tokens[1] == "get") {
              if (tokens[2] == "duty") {
                Serial.println( "motor1duty: " + String(motorList[0].getSetDuty()) );
                Serial.println( "motor2duty: " + String(motorList[1].getSetDuty()) );
                Serial.println( "motor3duty: " + String(motorList[2].getSetDuty()) );
              } else if (tokens[2] == "speed") {
                Serial.println( "motor1speed: " + String(motorList[0].getSpeed()) );
                Serial.println( "motor2speed: " + String(motorList[1].getSpeed()) );
                Serial.println( "motor3speed: " + String(motorList[2].getSpeed()) );
              } else if (tokens[2] == "mode") {
                Serial.println( "motor1mode: " + String(motorList[0].getControlMode()) );
                Serial.println( "motor2mode: " + String(motorList[1].getControlMode()) );
                Serial.println( "motor3mode: " + String(motorList[2].getControlMode()) );
              }

            }

          }
      }else if (token == "endEffect"){
        if (command != prevCommand) {

          // what am I interacting with to control the end effector? 
        }
      }else if (tokens[0] == "data") {                          
        if(command != prevCommand) {

          digitalWrite(LED_PIN, HIGH); 
          // Serial.print("Angle: ");                        // print some text to the serial consol.
          // Serial.println(encoder1.readAngleDegree());      // read the angle value from the AS5047P sensor an print it to the serial consol.
          // delay(500);                                     // wait for 500 milli seconds.
          data_bool = true; 
          
          // wait
          digitalWrite(LED_PIN, LOW);                     // deactivate the led.
          delay(500);
        } 
      }
      
      memset(allchars,'\0',sizeof(allchars));
      Serial.println(); 
      scommand = ""; 
      char_index = 0; 
    }//checkAxes(); 
  }
  if (data_bool) { //  change loop amount to change number of encoders checked
    for (int iter = 0; iter < 3; iter++) {
    Serial.print("Angle" + String(iter) + ": ");                        // print some text to the serial console.
    Serial.println(encoders[iter].readAngleDegree());      // read the angle value from the AS5047P sensor an print it to the serial consol.
    }
    delay(200);
  }
   
}



//-------------------------------------------------------//
//                                                       //
//    ///////////    //\\          //      //////////    //
//    //             //  \\        //    //              //
//    //             //    \\      //    //              //
//    //////         //      \\    //    //              //
//    //             //        \\  //    //              //
//    //             //          \\//    //              //
//    //             //           \//      //////////    //
//                                                       //
//-------------------------------------------------------//

// Magic that makes the SparkMax work with CAN? 
void loopHeartbeats(){
    myCan.begin();
    myCan.setBaudRate(1000000);
    myCan.setMaxMB(16);
    myCan.enableFIFO();
    myCan.enableFIFOInterrupt();

    while(1){
      sendHeartbeat(myCan, 1);
      threads.delay(5);
      sendHeartbeat(myCan, 2);
      threads.delay(5);
      sendHeartbeat(myCan, 3);
      threads.delay(5);
      threads.yield();
    }

}

// test function for trying to move the gat dang motors
// void test() { 
//   sendDutyCycle(myCan, Axis1.getID(),0.2); 
//   threads.delay(30); 
//   sendDutyCycle(myCan, Axis1.getID(),0.2); 
//   threads.delay(30); 
//   sendDutyCycle(myCan, Axis1.getID(),0.2); 
//   threads.delay(30); 
//   threads.yield(); 
// }



size_t posFlip(size_t pos, string delimiter, string scommand) { // function to change pos when specific conditions arise (deprecated)
  pos = scommand.find(delimiter);
  if (pos == size_t(-1)) {
    pos = scommand.length();
    return pos; 
  }
  else {return pos;}
}

// turn a motor for a specified amount of time
void turnTime(int time, // amount of time in ms to rotate for
              float duty, // speed/direction to rotate for
              int index)  // motor to rotate
{ 
  int startTime = millis();        // start of function runtime
  int newDuty = convertDuty(duty); // converted input duty

  if (index == 4) // if  all motors are selected (4)
  {
    for (int i = 0; i < (index-1); i++) // set duty cycle of all motors and run stepper
    {
      motorList[i].setDuty(newDuty);
      Axis0.run();
    }
  }
  else {
    motorList[index].setDuty(newDuty);
  }
  while (startTime - millis() <  u_long(time)){;} // wait the specified delay (ms)
  Serial.println("IT rain ig" + String()); 
  // threads.delay(time)
  for (int j = 0; j < 3; j++) // stop all motors and stepper motor
  {
    motorList[j].setDuty(convertDuty(0)); 
  }
  Axis0.stop();  
}

// split string for next token (deprecated)
void splitString(String &comm, String &prevComm, string tok, size_t pos, string scomm, string delim){
    scomm.erase(0, pos + delim.length()); 
    prevComm = comm; 
} 

// get the next token in string (deprecated)
void getToken(string token, size_t pos, string delimiter, string scommand){
    pos = scommand.find(delimiter); 
    token = scommand.substr(0,pos); 
    Serial.println(String(token.c_str())); 
}

// rotate stepper by steps indicated by stickValue
void moveStepper(float stickValue // movement speed indicator
){
  float speed = Axis0.speed(); 
  if (abs(stickValue) <= 0.02){
    Axis0.stop(); 
  }
  else 
  {
    if (stickValue > 1){
      Axis0.move(1); 
      Axis0.setSpeed(speed);
      Axis0.run(); 
    } else {
      Axis0.move(-1);
      Axis0.setSpeed(speed); 
      Axis0.run(); 
    }
  }
}

// approach limit
// approach inverse limit 
// approach zero

void movebyAngle(int angle, int axis, float duty) {
  ulong time = (angle/0.2)/15; 
  ulong start = millis(); 
  motorList[axis-1].setDuty(convertDuty(duty));
  while ((millis() - start) < time*1000) {
    //Serial.println(start - millis()); 
    if(millis()-lastAccel >= 50){
    lastAccel = millis();
    for(int i = 0; i < 4; i++) {
      motorList[i].UpdateForAcceleration();
    }

    if(motorList[0].getControlMode() == 1)//send the correct duty cycle to the motors
    {
        for(int i = 0; i < 4; i++)
        {
          sendDutyCycle(myCan, motorList[i].getID(), motorList[i].getDuty()); // update motors with current duty cycle
        }
    } else{
        //pass for RPM control mode
    }
  }

  } 
  //Serial.println(String(start - millis())); 
  //Serial.println(String(time*1000)); 
}

// move axis in reverse direction of possible collision
void moveDirection(int mode // forwards or backwards movement?
) {

  for (int i = 0; i < motor_num; i++) { // if angle too high, move back
    if (getAngle(i) < 0.5){
      moveAxis(backDuty,i);
    }
    else if (getAngle(i) > 0.5) { // else, move forward
      moveAxis(forwardDuty,i);
    }
  }
} 


// move a singular axis by duty 'stickValue' and index
void moveAxis(float stickValue, // duty cycle to move with
              int index         // index of selected motor
 ){
  motorList[index].setDuty(convertDuty(stickValue)); // move, then check axes for limit
  checkAxes(); 
}

void checkAxes() { // check if axes are over defined limits 
  for (int i = 0; i < motor_num; i++) {
    if(abs(getAngle(i)) >= limits[i])
    {
      motorList[i].setDuty(convertDuty(0)); 
    }
  }
}

// get angle from magnetic envoder for specified axis
int getAngle(int index) {  
 // waiting
 return 0; 
}

// convert duty cycle given to appropriate for motors
float convertDuty(float duty // duty cycle to give
) {
  if (abs(duty) <= 0.02) { // coerce minimal values to zero
    return 0; 
  }
  if (duty > 0) // coerce duty to predefined forward movement value
  {
    lastDuty = forwardDuty*duty; 
    return forwardDuty; 
  }
  else if (duty < 0) // coerce duty to predefined backward movement value
  {
    lastDuty = backDuty*abs(duty); 
    Serial.println(backDuty); 
    return backDuty; 
  }
  return duty; 
}

// int overload for convertDuty(float duty)
float convertDuty(int duty) {
  if (abs(duty) <= 0.02) {
    return 0; 
  }
  if (duty > 0)
  {
    lastDuty = forwardDuty*duty; 
    return forwardDuty; 
  }
  else if (duty < 0)
  {
    lastDuty = backDuty*abs(duty); 
    return backDuty; 
  }
  return float(duty); 
}

// dump character array tokens into string format
void strdump(char* strings[COMMAND_LIMIT], // input tokens to convert
             string result[COMMAND_LIMIT]) // output array of strings
 {
    int i = 0; 
    while (strings[i]) { // convert and place new string into result array if not blank
        result[i] = string(strings[i]); 
        i++;
    }
}

// pump tokens into character array for proccessing
void charpump(char* tokes[COMMAND_LIMIT], // output character array for further conversion
              string str) { // input string

    char* tok_collect = new char[str.length() + 1]; // define character array for copying
    strcpy(tok_collect,str.c_str());                // copy character string of input into character array
    tok_collect = strtok(tok_collect,",");          // split character array into tokens with splitter ","
    int i = 0; 
    while (tok_collect != NULL){                    // while tokens remain in character array, pipe into output array
        tokes[i] = tok_collect;                     
        tok_collect = strtok(NULL,","); 
        i++; 
    }
}

// fix input string depending on method of serial input 
void fixString(string& str  // string to fix
             , bool &fix) { // value determining whether to fix
    if (!fix) { // if first input has been given, null character at start of string, need to remove
        fix = true;
    }
    else { // if null character exists, shift input one space to the left. 
        string temp;
        for (int j = 0; j < int(str.length()-1); j++)
        {
            temp += str[j+1]; // move elements of string left
        }  
        str = temp; // replace original string with left-shifted string
    }
}