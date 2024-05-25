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

//test
#include <CrashReport.h>

using namespace std;

#define LSS_BAUD    (LSS_DefaultBaud)
#define LSS_SERIAL    (Serial7)

#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)


//Setting up for myCan line
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;

//Setting up for magnetic encoders 
AS5047P encoder1(10, AS5047P_CUSTOM_SPI_BUS_SPEED); 
AS5047P encoder2(37, AS5047P_CUSTOM_SPI_BUS_SPEED);
AS5047P encoder3(36, AS5047P_CUSTOM_SPI_BUS_SPEED);
AS5047P encoders[3] = {encoder1, encoder2, encoder3}; 

float cpu_temp = tempmonGetTemp();
bool led_state = false;

//Setting arm!
//AccelStepper Axis0(AccelStepper::FULL2WIRE,2,3,4,5); // Axis0 belt-driven NEMA stepper
//HighPowerStepperDriver Axis_0; // which to use?

AstraMotors Axis1(1, 1, false, 50, 0.50F);//40:1 cycloidal
AstraMotors Axis2(2, 1, false, 50, 0.50F);//20:1 cycloidal
AstraMotors Axis3(3, 1, false, 50, 0.50F);//12:1 cycloidal
AstraMotors motorList[3] = {Axis1, Axis2, Axis3};

AstraWrist wrist(90,20);//Wrist object, set max tilt to +-90 degrees

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

//int axis0_state = 0; // Axis 0 state (0: stop, 1: cw, -1: ccw)
int x0_state = 0; // Axis 0 state (0: stop, 1: cw, -1: ccw)
int x0_update_time_ms = 125;//update time for axis 0
Servo axis0; // Axis 0 servo object


unsigned long lastCtrlCmd;//last time a control command was received. Used for safe stop if control is lost
unsigned long lastMotorUpdate;//last time the motors were updated
unsigned long lastFeedback;//last time feedback was sent
unsigned long lastx0;//last time axis 0 was updated
unsigned long lastWrist;//last time the wrist was moved
int rotate_time_ms = 175;

// Function prototypes 
void loopHeartbeats(); //heartbeat for sparkmax controllers
void cmd_check(); //check for command if data is in the serial buffer
void parseInput(const String input, std::vector<String>& args, char delim); // parse command to args[]
void update_x0();//Move axis 0 based on its state
void safety_timeout();//stop all motors if no control commands are received for a certain amount of time
void EF_manip();//manipulate the end effector based on its states
void update_motors();//send duty cycle control command to all motors
void feedback();//send feedback (state) on usb serial line
void move_wrist(bool revolve, bool invert);//move the wrist based on its states



void setup() {

    //built_in teensy LED
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);//usb serial line
    digitalWrite(LED_PIN, HIGH);

    //blink to signify boot
    delay(2000);
    digitalWrite(LED_PIN, LOW);

    Serial3.begin(115200);//Digit board serial line

  // Check and print crash report if available
  if (CrashReport) {
    Serial.print(CrashReport);
  }else
  {
    Serial.println("No crash report available");
  }

  //*((volatile uint32_t *)0) = 0;


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
    axis0.attach(19);

  //Setup wrist LSS motors
    LSS::initBus(LSS_SERIAL, LSS_BAUD);
    top_lss.setMaxSpeed(100);
    bottom_lss.setMaxSpeed(100);
  
    //top_lss.move(0);
    //bottom_lss.move(0);


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
    cmd_check(); //check for command if data is in the serial buffer
  }

  if(Serial3.available()){
    String command = Serial3.readStringUntil('\n');
    command.trim();
    if(command.length() > 11)
      if(command.substring(0,10) == "faeriesht,")
        Serial.println(command);
  }

  safety_timeout();//stop all motors if no control commands are received for a certain amount of time (3 seconds)
  update_x0();//move Axis_0 based on its state
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
  //Serial.println("GOT COMMAND");

  String command = Serial.readStringUntil('\n');  
  command.trim();
                      
  std::vector<String> args = {};
  parseInput(command, args, ',');
  //Serial.println("Got command: " + command);

  
  //Serial.printf("args: %s, %s, %s, %s\n", args[0].c_str(), args[1].c_str(), args[2].c_str(), args[3].c_str());

    if (args[0] == "arm") { // Is looking for a command -> "arm,..."

      if(args[1] == "setMode"){//arm,setMode,mode

        if(args[2] == "ik"){//arm,setMode,ik
          ik_mode = true;
          //Serial.println("Set control mode to: IK");
        }else if(args[2] == "manual"){//arm,setMode,manual
          ik_mode = false;
          //Serial.println("Set control mode to: manual");
        }else{
          Serial.println("Invalid control mode!");
        }

      }else if(args[1] == "man"){//arm,man,duty,axis_0,axis_1,axis_2,axis_3. // -1: ccw, 0: stop, 1: cw 
        if(ik_mode)
        {
          Serial.println("Manual control disabled while in IK mode");
        }else if(args.size() == 7)//if not in IK mode and got enough arguments for all joints
        {
          x0_state = args[3].toInt();//set axis0 state

          motorList[0].setDuty(args[2].toFloat()*args[4].toFloat()*2.1);//axis1, 2.1x speed multiplier
          delay(1);
          motorList[1].setDuty(args[2].toFloat()*args[5].toFloat()*1.4);//axis2, 4.4x speed multiplier (higher multiplier because in brake mode) (originally 1.4, up to 4.4 for brake, pulled back down to 2.22)
          delay(1);
          motorList[2].setDuty(args[2].toFloat()*args[6].toFloat()*0.9);//axis3, 0.9x speed multiplier

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
          x0_state = args[2].toInt();//set axis0 state
          //Serial.println("trying to plan IK...");
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
        //if((args[2] == "ctrl" && args.size() != 6) || (args[2] == "laser" && args.size() != 4))
        //{
        //  Serial.println("Invalid number of args for endEffect command");
        //}else
        //{
          if(args[2] == "ctrl")//arm,endEffect,ctrl,gripper_state,tilt_state,rotation_state
          {
            //Serial.printf("Sending: digit,%s,%d\n",args[2].c_str(),args[3].toInt());
            Serial3.printf("digit,ctrl,%d\n",args[3].toInt()); //send gripper control command to digit board
            //Serial.printf("digit,ctrl,%d\n",args[3].toInt());
            wrist_tilt_state = args[4].toInt();//set wrist tilt state
            wrist_revolve_state = args[5].toInt();//set wrist revolve state

            //rotate_time_ms = args[6].toInt();//set the time for the wrist movement, this should probably just be used for testing
            lastWrist = millis();//update last wrist command time

          }else if(args[2] == "laser"){
            Serial3.printf("digit,laser,%d\n",args[3].toInt());//send laser control command to digit board
            //Serial.printf("digit,laser,%d\n",args[3].toInt());//send laser control command to digit board
          }
        //}
      }else if (args[1] == "axis"){ // "arm,axis,axis_#,duty_cycle" // controls a single axis at a time (set direction)// FOR TESTING ONLY

        if(args[2] == "0"){//different control scheme for axis0
          x0_state = args[3].toInt();//set state of axis 0

        }else if ((args[2].toInt() >= 1) && (args[2].toInt()<= 3)){//ensure the axis is a valid index
          int motor_index = args[2].toInt() - 1; //get the motor id (-1 for index)

          motorList[motor_index].setDuty(args[3].toFloat());
          //sendDutyCycle(myCan, motorList[motor_index].getID(), motorList[motor_index].getDuty()); // update motors with current duty cycle
        }
        lastCtrlCmd = millis();//update last control command time

      }else if(args[1] == "stop") { // "arm,stop" //stops movement on all axis
        Serial.println("Stopping all motors");

        for (int j = 0; j < 3; j++) {
          motorList[j].setDuty(0);
          sendDutyCycle(myCan, motorList[j].getID(), 0.0); // stop all motors
        }
        
        x0_state = 0; //axis 0 movement to stop

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
      }else if (args[1] == "aux")
      {
        if(args[2] == "lynx_reset")
        {
          Serial.println("Resetting Lynxmotion LSS bus");
          top_lss.reset();
          bottom_lss.reset();
        }
      }

      
    }else if (args[0] == "endEffect"){// looking for a command -> "endEffect,..."
      //pass
    }else if (args[0] == "faerie") {
      Serial3.println(command);
      Serial.printf("Sending to faerie: %s\n", command.c_str());
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


void update_x0()
{
  if(millis() - lastCtrlCmd <= x0_update_time_ms)
  {
    if(x0_state == 0)
    {
      axis0.writeMicroseconds(1500);//stop
      //Serial.println("STOPPING AXIS 0");
    }else if(x0_state == 1)
    {
      axis0.writeMicroseconds(1100);//cw
      //Serial.println("MOVING AXIS 0 CW");
    }else if(x0_state == -1)
    {
      axis0.writeMicroseconds(1900);//ccw
      //Serial.println("MOVING AXIS 0 CCW");
    }
  }else{
    axis0.writeMicroseconds(1500);//stop
  }
}

///*
void loopHeartbeats(){//provide heartbeat for spark max controllers
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

}//*/


void feedback()
{
  if(millis() - lastFeedback > 1000)//Send out the arm's status every 2 seconds
  {
    //Serial.printf("feedback,%f,%f,%f,%d,%d,%d,%d\n",arm.angles[0],arm.angles[1],arm.angles[2],arm.wrist.cur_tilt,wrist_revolve_state,wrist_tilt_state,x0_state);
    cpu_temp = tempmonGetTemp();
    Serial.printf("feedback,arm,temp,%f\n", cpu_temp);
    
    lastFeedback = millis();
    
    if(cpu_temp >= 60.0)
    {
      Serial.println("******************************");
      Serial.printf("CPU TEMP IS TOO HIGH: %f\n", cpu_temp);
      Serial.println("******************************");
    }
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
    x0_state = 0;//stop axis 0

    arm.ik_obj.active = false;//disable IK objective to stop IK movement
  }
}


void EF_manip(){
  //manipulate the end effector based on its states
  //wrist_tilt_state = 0; // Wrist tilt state (0: stop, 1: right, -1: left)
  //wrist_revolve_state = 0; // Wrist revolve state (0: stop, 1: cw, -1: ccw)
  if ((millis() - lastWrist) <= rotate_time_ms) {
    if(wrist_tilt_state == 0 && wrist_revolve_state == 0)
    {
      top_lss.wheelRPM(0);
      bottom_lss.wheelRPM(0);
      return;
    }else if(wrist_tilt_state != 0 && wrist_revolve_state != 0)//give preference to revolve
    {
      wrist_tilt_state = 0;//stop tilt if revolve is active
    }
    if(wrist_tilt_state != 0)
    {
      //manipulate wrist tilt
      if(wrist_tilt_state >= 1)//left
      {
        move_wrist(0, 1);
      }else if(wrist_tilt_state <= -1)//right
      {
        move_wrist(0, 0);
      }
    }else if(wrist_revolve_state != 0)
    {
      //manipulate wrist revolve
      if(wrist_revolve_state >= 1)//ccw
      {
        move_wrist(1, 1);
      }else if(wrist_revolve_state <= -1)//cw
      {
        move_wrist(1, 0);
      }
    } 
  } else { 
    top_lss.wheelRPM(0); 
    bottom_lss.wheelRPM(0);
    //top_lss.reset();
    //bottom_lss.reset();
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
void move_wrist(bool revolve, bool invert){
    float angle = wrist.step_size * 10;//convert to 0.1 degree increments
    Serial.println("RUNNING MOVE_WRIST");
    if(invert)
    {
        angle *= -1;//tilt left / revolve ccw
    }

    if(revolve){//if true, revolve (opposite absolute directions)
        top_lss.moveRelative(angle*3);
        bottom_lss.moveRelative(angle*3);
    }else{
        if(wrist.cur_tilt + angle > wrist.max_tilt || wrist.cur_tilt+angle < (wrist.max_tilt * -1)){
            return;//max angle would be exceeded, don't move the motors
        }
        top_lss.moveRelative(angle);
        bottom_lss.moveRelative(angle*-1);
        wrist.cur_tilt += angle;
    }
}
*/

void move_wrist(bool revolve, bool invert){
    int speed = wrist.rpm;//convert to 0.1 degree increments
    if(invert)
    {
        speed *= -1;//tilt left / revolve ccw
    }

    if(revolve){//if true, revolve (opposite absolute directions)
        top_lss.wheelRPM(speed);
        bottom_lss.wheelRPM(speed);
    }else{
        /*if(wrist.cur_tilt + angle > wrist.max_tilt || wrist.cur_tilt+angle < (wrist.max_tilt -1)){
            return;//max angle would be exceeded, don't move the motors
        }*/
        top_lss.wheelRPM(speed*.25);
        bottom_lss.wheelRPM(speed*-1*.25);
        //wrist.cur_tilt += speed;
    }
} 



