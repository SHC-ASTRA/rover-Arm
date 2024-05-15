// Standard Includes
#include <Arduino.h>
#include <iostream>
#include <string>

// Our own resources
#include "AstraMotors.h"
#include "AstraArm.h"
#include "AstraCAN.h"
#include "AstraSensors.h"
#include "TeensyThreads.h"

using namespace std;


#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)


//Setting up for myCan line
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;

//Setting up for magnetic encoders 
AS5047P encoder1(10, AS5047P_CUSTOM_SPI_BUS_SPEED); 
AS5047P encoder2(37, AS5047P_CUSTOM_SPI_BUS_SPEED);
AS5047P encoder3(36, AS5047P_CUSTOM_SPI_BUS_SPEED);
AS5047P encoders[3] = {encoder1, encoder2, encoder3}; 


//Setting arm!
AccelStepper Axis0(AccelStepper::FULL2WIRE,2,3,4,5); // Axis0 belt-driven NEMA stepper
HighPowerStepperDriver Axis_0; // which to use?

AstraMotors Axis1(1, 1, false, 50, 0.50F);//40:1 cycloidal
AstraMotors Axis2(2, 1, false, 50, 0.50F);//20:1 cycloidal
AstraMotors Axis3(3, 1, false, 50, 0.50F);//12:1 cycloidal
AstraMotors motorList[3] = {Axis1, Axis2, Axis3};

AstraWrist wrist(90,0.5);//Wrist object, set max tilt to +-90 degrees

LSS top_lss = LSS(1);     //top LSS on wrist
LSS bottom_lss = LSS(2);  //bottom LSS on wrist


int arm_segments[3] = {100,100,100}; //Length of each arm segment (units: mm)
int arm_ratios[3] = {40,20,12}; //Joint gear ratios (multiplier)
float arm_angles[3] = {0.0,0.0,0.0}; //Current joint angles (units: degrees)
float arm_cur_pos[2] = {0.0,0.0}; //Current end effector position (units: mm)
AstraArm arm(arm_segments, arm_ratios, arm_angles, arm_cur_pos);//Arm object

//Vars
bool ik_mode = false; // Inverse kinematics control mode

int wrist_tilt_state = 0; // Wrist tilt state (0: stop, 1: right, -1: left)
int wrist_revolve_state = 0; // Wrist revolve state (0: stop, 1: cw, -1: ccw)

int axis0_state = 0; // Axis 0 state (0: stop, 1: cw, -1: ccw)
const uint8_t DirPin = 2;
const uint8_t StepPin = 3;
const uint8_t CSPin = 4;
const uint16_t StepPeriodUs = 2000;// This period is the length of the delay between steps, which controls the stepper motor's speed

unsigned long lastCtrlCmd;//last time a control command was received. Used for safe stop if control is lost
unsigned long lastMotorUpdate;//last time the motors were updated
unsigned long lastFeedback;//last time feedback was sent

// Function prototypes 
void loopHeartbeats(); //provide heartbeat to spark max controllers
void cmd_check(); //check for command if data is in the serial buffer
void parseInput(const String input, std::vector<String>& args, char delim); // parse command to args[]
void step_x0();//Step the axis0 motor when necessary
void safety_timeout();//stop all motors if no control commands are received for a certain amount of time
void EF_manip();//manipulate the end effector based on its states
void update_motors();//send duty cycle control command to all motors
void feedback();//send feedback (state) on usb serial line



void setup() {

    //built_in teensy LED
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);//usb serial line
    digitalWrite(LED_PIN, HIGH);

    //blink to signify boot
    delay(2000);
    digitalWrite(LED_PIN, LOW);

    Serial3.begin(115200);//Digit board serial line


  //-----------------//
  //   Initialize    //
  //-----------------//

  //Setup CAN bus
    myCan.begin(); // Initialize CAN bus settings
    myCan.setBaudRate(1000000);
    myCan.setMaxMB(16);
    myCan.enableFIFO();
    myCan.enableFIFOInterrupt();

  //Setup axis 0
    SPI.begin();//start SPI for stepper motor and encoders
    Axis_0.setChipSelectPin(CSPin);
    pinMode(StepPin, OUTPUT); // Drive the STEP and DIR pins low initially.
    digitalWrite(StepPin, LOW);
    pinMode(DirPin, OUTPUT);
    digitalWrite(DirPin, LOW);
    delay(1);     // Give the driver some time to power up.
    Axis_0.resetSettings();    // Reset the driver to its default settings and clear latched status conditions.
    Axis_0.clearStatus();     
    Axis_0.setDecayMode(HPSDDecayMode::AutoMixed);     // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode for most applications, and we find that it usually works well.
    Axis_0.setCurrentMilliamps36v4(1000);     // Set the current limit. You should change the number here to an appropriate value for your particular system.
    Axis_0.setStepMode(HPSDStepMode::MicroStep32);    // Set the number of microsteps that correspond to one full step.
    Axis_0.enableDriver();      // Enable the motor outputs.


  //Setup wrist LSS motors
    LSS::initBus(LSS_SERIAL, LSS_BAUD);
    top_lss.setMaxSpeed(100);
    bottom_lss.setMaxSpeed(100);
  
    top_lss.move(0);
    bottom_lss.move(0);


  //--------------------//
  // Initialize Sensors //
  //--------------------//
 
  //Start heartbeat thread
  //TEMPORARY FIX, until we get a dedicated microcontroller for heartbeat propogation
  threads.addThread(loopHeartbeats);
  //threads.addThread(test); 

  
  for(int e = 0; e < 3; e++)
  {
    if(!encoders[e].initSPI()) {
      Serial.printf("Can't connect to AS5047P sensor %d!\n",e+1);
      delay(1000);
    }else{
      Serial.printf("Found AS5047P sensor %d!\n",e+1);
    }
  }

  /*
  while(1){
    Serial.printf("Encoders:\n");
    for(int e = 0; e < 3; e++)
    {
      Serial.printf("%d: %f\n",e+1,encoders[e].readAngleDegree());
    }
    Serial.println("\n\n");
    delay(500);
  }*/
  
}



void loop(){
  
  if(Serial.available()){
    Serial.println("Serial Recieved...");
    cmd_check(); //check for command if data is in the serial buffer
  }

  safety_timeout();//stop all motors if no control commands are received for a certain amount of time (3 seconds)
  //step_x0();//move Axis_0 based on its state
  //arm.IK_Execute();//execute the inverse kinematics for the arm (update arm speeds,angles,pos,etc..)
  EF_manip();//move end effector based on its states
  update_motors();//Move the motors their given direction
  feedback();//send feedback (state) on usb serial line
}




//Functions Section
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
//Functions Section



void cmd_check(){
  if (Serial.available()) {//double check just for good measure

  String command = Serial.readStringUntil('\n');  
  command.trim();
                      
  std::vector<String> args = {};
  parseInput(command, args, ',');

  
  //Serial.printf("args: %s, %s, %s, %s\n", args[0].c_str(), args[1].c_str(), args[2].c_str(), args[3].c_str());

    if (args[0] == "arm") { // Is looking for a command -> "arm,..."

      if(args[1] == "setMode"){//arm,setMode,mode

        if(args[2] == "ik"){//arm,setMode,ik
          ik_mode = true;
          Serial.println("Set control mode to: IK");
        }else if(args[2] == "manual"){//arm,setMode,manual
          ik_mode = false;
          Serial.println("Set control mode to: manual");
        }else{
          Serial.println("Invalid control mode!");
        }

      }else if(args[1] == "man"){//arm,man,duty,axis_0,axis_1,axis_2,axis_3. // -1: ccw, 0: stop, 1: cw 
        if(ik_mode)
        {
          Serial.println("Manual control disabled while in IK mode");
        }else if(args.size() == 7)//if not in IK mode and got enough arguments for all joints
        {
          axis0_state = args[2].toInt();//set axis0 state
          
          Serial.println("trying to set motor duty cycles");

          for(int m = 0; m < 3; m++)
          {
            motorList[m].setDuty(args[2].toFloat()*args[m+4].toFloat());//set duty cycle to and provided direction (or stop)
            //sendDutyCycle(myCan, motorList[m].getID(), motorList[m].getDuty());//send duty cycle to motor
            delay(2);//delay to ensure all commands go through on CAN
          }

          lastCtrlCmd = millis();//update last control command time
        }else{
          Serial.println("Manual control error: invalid number of inputs");
        }
      }else if(args[1] == "ik")//arm,ik,axis_0_state,rel_target_x,rel_target_y
      {
        if(!ik_mode){
          Serial.println("IK control disabled while in manual mode");
        }else if(args.size() == 5)//if in IK mode and correct number of arguments
        {
          axis0_state = args[2].toInt();//set axis0 state
          Serial.println("trying to plan IK...");
          int ik_output = arm.IK_Plan(args[3].toFloat(), args[4].toFloat());//plan the IK movement
          if(ik_output == 1)
          {
            lastCtrlCmd = millis();//update last control command time
          }else{//IK Failed planning
            Serial.println("IK planning failed");
          }
        }else{ 
          Serial.println("IK control error: invalid number of inputs");
        }
      }else if(args[1] == "endEffect")//arm,endEffect,...
      {
        if(args[2] == "ctrl")//arm,endEffect,ctrl,gripper_state,tilt_state,rotation_state
        {
          Serial3.printf("digit,ctrl,%d",args[3].toInt()); //send gripper control command to digit board
          
          wrist_tilt_state = args[4].toInt();//set wrist tilt state
          wrist_revolve_state = args[5].toInt();//set wrist revolve state

        }else if(args[2] == "laser"){
          Serial3.printf("digit,laser,%d",args[3].toInt());//send laser control command to digit board
        }
      }else if (args[1] == "axis"){ // "arm,axis,axis_#,duty_cycle" // controls a single axis at a time (set direction)// FOR TESTING ONLY

        if(args[2] == "0"){//different control scheme for axis0

        axis0_state = args[3].toInt();//set state of axis 0

        }else if ((args[2].toInt() >= 1) && (args[2].toInt()<= 3)){//ensure the axis is a valid index
          int motor_index = args[2].toInt() - 1; //get the motor id (-1 for index)

          motorList[motor_index].setDuty(args[3].toFloat());
          //sendDutyCycle(myCan, motorList[motor_index].getID(), motorList[motor_index].getDuty()); // update motors with current duty cycle
        }

      }else if(args[1] == "stop") { // "arm,stop" //stops movement on all axis
        Serial.println("Stopping all motors");

        for (int j = 0; j < 3; j++) {
          motorList[j].setDuty(0);
          sendDutyCycle(myCan, motorList[j].getID(), 0.0); // stop all motors
        }
        
        axis0_state = 0; //axis 0 movement to stop

      } else if  (args[1] == "ping") { // "arm,ping"

        Serial.println("pong"); 
      } else if  (args[1] == "time")  { 
        //command not currently used
      } else if  (args[1] == "test") {
        //command not currently used
      } else if (args[1] == "get") {//arm,get,duty ; arm,get,speed ; arm,get,angle
        if (args[2] == "duty") {
          Serial.println( "motor1duty: " + String(motorList[0].getSetDuty()) );
          Serial.println( "motor2duty: " + String(motorList[1].getSetDuty()) );
          Serial.println( "motor3duty: " + String(motorList[2].getSetDuty()) );
        } else if (args[2] == "speed") {
          Serial.println( "motor1speed: " + String(motorList[0].getSpeed()) );
          Serial.println( "motor2speed: " + String(motorList[1].getSpeed()) );
          Serial.println( "motor3speed: " + String(motorList[2].getSpeed()) );
        }else if (args[2] == "angle")
        {
          //To Be Implemented
        }

      }

      
    }else if (args[0] == "endEffect"){// looking for a command -> "endEffect,..."
      //pass
    }else if (args[0] == "data") {  
      /*
        digitalWrite(LED_PIN, HIGH); 
        // Serial.print("Angle: ");                        // print some text to the serial consol.
        // Serial.println(encoder1.readAngleDegree());      // read the angle value from the AS5047P sensor an print it to the serial consol.
        // delay(500);                                     // wait for 500 milli seconds.
        data_bool = true; 
        
        // wait
        digitalWrite(LED_PIN, LOW);                     // deactivate the led.
        delay(500);
        */
    }
    
  }//checkAxes(); 
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

void step_x0()
{
  if(axis0_state != 0)//if not stop
  {
    delayMicroseconds(1);//need delay before & after to set direction
    if(axis0_state >= 1)//positive for cw
      {digitalWrite(DirPin, axis0_state);}//input 1 results in cw motion
    else //negative for ccw
      {digitalWrite(DirPin, 0);}//convert the -1 input to 0 for ccw motion
    delayMicroseconds(1);

    // The STEP minimum high pulse width is 1.9 microseconds.
    digitalWrite(StepPin, HIGH);
    delayMicroseconds(3);
    digitalWrite(StepPin, LOW);
    delayMicroseconds(3);
  }

}


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


void feedback()
{
  if(millis() - lastFeedback > 2000)//Send out the arm's status every 2 seconds
  {
    Serial.printf("feedback,%f,%f,%f,%f\n",arm.angles[0],arm.angles[1],arm.angles[2],arm.wrist.cur_tilt);
  }
}

void safety_timeout(){
  if(millis() - lastCtrlCmd > 3000)//if no control commands are received for 3 seconds
  {
    for(int m = 0; m < 3; m++)
    {
      motorList[m].setDuty(0);//stop all motors
      sendDutyCycle(myCan, motorList[m].getID(), 0);//send stop command to all motors
    }
    axis0_state = 0;//stop axis 0

    arm.ik_obj.active = false;//disable IK objective to stop IK movement
  }
}


void EF_manip(){
  //manipulate the end effector based on its states
  //wrist_tilt_state = 0; // Wrist tilt state (0: stop, 1: right, -1: left)
  //wrist_revolve_state = 0; // Wrist revolve state (0: stop, 1: cw, -1: ccw)

  if(wrist_tilt_state != 0 && wrist_revolve_state != 0)//give preference to revolve
  {
    wrist_tilt_state = 0;//stop tilt if revolve is active
  }
  if(wrist_tilt_state != 0)
  {
    //manipulate wrist tilt
    if(wrist_tilt_state >= 1)//right
    {
      move_wrist(wrist, 0, 0, top_lss, bottom_lss);
    }else if(wrist_tilt_state <= -1)//left
    {
      move_wrist(wrist, 0, 1, top_lss, bottom_lss);
    }
  }else if(wrist_revolve_state != 0)
  {
    //manipulate wrist revolve
    if(wrist_revolve_state >= 1)//cw
    {
      move_wrist(wrist, 1, 0, top_lss, bottom_lss);
    }else if(wrist_revolve_state <= -1)//ccw
    {
      move_wrist(wrist, 1, 1, top_lss, bottom_lss);
    }
  }


}





void update_motors()
{
  if(millis()-lastMotorUpdate > 500)
  {
    for(int j = 0; j < 3; j++)
    {
      sendDutyCycle(myCan, motorList[j].getID(), motorList[j].getDuty());
    }
    lastMotorUpdate = millis();
  }
}


/*


#define SEALEVELPRESSURE_HPA (1013.25)






//AstraMotors(int setMotorID, int setCtrlMode, bool inv, int setMaxSpeed, float setMaxDuty)

//AstraMotors Axis3(3, 1, false, 50, 0.50F);//BR

//Left motors first, Right motors Second


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

void   strdump(char* strings[COMMAND_LIMIT], string result[COMMAND_LIMIT]); // dump character array contents into string array
void   fixString(string& str , bool &fix); // fix incoming string when using VSCode serial monitor
//void   test(); // nothing lives and breathes to the fullest extent









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

void cmd_check(){
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
}






  if (data_bool) { //  change loop amount to change number of encoders checked
    for (int iter = 0; iter < 3; iter++) {
    Serial.print("Angle" + String(iter) + ": ");                        // print some text to the serial console.
    Serial.println(encoders[iter].readAngleDegree());      // read the angle value from the AS5047P sensor an print it to the serial consol.
    }
    delay(200);
  }
   
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

*/