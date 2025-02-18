/**
 * @file Main.cpp
 * @author Charles Marmann (cmm0077@uah.edu)
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

#include <SPI.h>
#include <HighPowerStepperDriver.h>

// Our own resources

#include "project/ARM.h"

//#include "project/CORE.h"

#include "AstraMisc.h"
#include "AstraVicCAN.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK


//Sensor declarations

AstraCAN Can0;


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

const uint16_t StepPeriodUs = 2000;
HighPowerStepperDriver sd;

unsigned long clockTimer = 0;
unsigned long lastFeedback = 0;
unsigned long lastMotorStep = 0;
unsigned long lastCtrlCmd = 0;
double AX0Speed = 0;
bool AX0En = false;

unsigned int goalTime;

bool AxisComplete    [] = {true,true,true,true};     // AxisXComplete    where x = 1..3
int  AxisSetPosition [] = {0,0,0,0};              // AxisXSetPosition ^^^
int  AxisPosition    [] = {0,0,0,0};              // AxisXPosition    ^^^


//--------------//
//  Prototypes  //
//--------------//

void outputEncoders();
void safety_timeout();
void findSpeedandTime(int time);
void convertToDutyCycle(double& dpsSpeed, float gearRatio);
void convertToDutyCycleA0(double& dpsSpeed, float gearRatio);
void updateMotorState();

// int findRotationDirection(float current_direction, float target_direction);
// bool autoTurn(int time,float target_direction);
//void setLED(int r_val, int b_val, int g_val);


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

    if(ESP32Can.begin(TWAI_SPEED_1000KBPS, 24, 25))
        Serial.println("CAN bus started!");
    else
        Serial.println("CAN bus failed!");
    
    vicCAN.relayOn();

    sd.setChipSelectPin(AX0_CS);

    delay(1);

    sd.resetSettings();
    sd.clearStatus();

    // Select auto mixed decay.  TI's DRV8711 documentation recommends this mode
    // for most applications, and we find that it usually works well.
    sd.setDecayMode(HPSDDecayMode::AutoMixed);

    // Set the current limit. You should change the number here to an appropriate
    // value for your particular system.  If you are using a 36v8 board, call
    // setCurrentMilliamps36v8 instead.
    sd.setCurrentMilliamps36v4(1000);

    // Set the number of microsteps that correspond to one full step.
    sd.setStepMode(HPSDStepMode::MicroStep1);

    // Enable the motor outputs.
    sd.enableDriver();
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

    if(((millis()-lastMotorStep)>=4) && AX0En)
    {
        sd.step();
        lastMotorStep = millis();
    }

    if((millis()-lastFeedback)>=2000)
    {
        
        lastFeedback = millis();
    }

    // Safety timeout if no ctrl command for 2 seconds
    safety_timeout();


    //------------------//
    //  CAN Input  //
    //------------------//
    //
    //
    //-------------------------------------------------------//
    //                                                       //
    //      /////////          //\\          //\\      //    //
    //    //                  //  \\         // \\     //    //
    //    //                 //    \\        //  \\    //    //
    //    //                /////\\\\\       //   \\   //    //
    //    //               //        \\      //    \\  //    //
    //    //              //          \\     //     \\ //    //
    //      /////////    //            \\    //      \\//    //
    //                                                       //
    //-------------------------------------------------------//

    if(vicCAN.readCan()) {
        const uint8_t commandID = vicCAN.getCmdId();
        static std::vector<double> canData;
        vicCAN.parseData(canData);

        #ifdef DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.print("| Main MCU VicCAN Recieved: ");
            Serial.print(commandID);
            Serial.print("; ");
            if (canData.size() > 0) {
                for (const double& data : canData) {
                    Serial.print(data);
                    Serial.print(", ");
                }
            }
            Serial.println();
        #endif

        // Misc

        /**/ if (commandID == CMD_PING) {
            vicCAN.respond(1);  // "pong"
            // Serial.println("Received ping over CAN");
        }
        else if (commandID == CMD_B_LED) {
            if (canData.size() == 1) {
                if (canData[0] == 0)
                    digitalWrite(LED_BUILTIN, false);
                if (canData[0] == 1)
                    digitalWrite(LED_BUILTIN, true);
            }
        }

        // REV

        else if (commandID == CMD_REV_STOP) {
            COMMS_UART.println("Stop");
        }
        else if (commandID == CMD_REV_IDENTIFY) {
            if (canData.size() == 1) {
                COMMS_UART.print("rev_id,");
                COMMS_UART.println(canData[0]);
            }
        }
        else if (commandID == CMD_REV_IDLE_MODE) {
            if (canData.size() == 1) {
                if (canData[0] == 0)
                    COMMS_UART.println("brake,on");
                else if (canData[0] == 1)
                    COMMS_UART.println("brake,off");
            }
        }
        else if (commandID == CMD_ARM_IK_CTRL) {
            if (canData.size() == 4) {
                #ifdef DEBUG
                    Serial.println("|------------------------------------------------------|");
                    Serial.println("| VicCan IK Angle cmd recieved                         |");
                #endif
                for (int i = 0; i < MOTOR_AMOUNT; i++) 
                {
                    AxisSetPosition[i] = canData[i];
                }
            }
        }
        else if (commandID == CMD_ARM_IK_TTG) {
            if (canData.size() == 1) {
                #ifdef DEBUG
                    Serial.println("|------------------------------------------------------|");
                    Serial.println("| VicCan IK Time cmd recieved                          |");
                #endif
                findSpeedandTime(canData[0]);
            }
        }
        else if (commandID == CMD_ARM_MANUAL) {
            if (canData.size() == 4) {

                #ifdef DEBUG
                    Serial.println("|------------------------------------------------------|");
                    Serial.println("| VicCan ctrl cmd recieved                             |");
                #endif
                

                float speeds[3];
                speeds[0] = canData[1] * 0.50;
                speeds[1] = canData[2] * 0.50;
                speeds[2] = canData[3] * 0.50;
                String command = "ctrl," + String(speeds[0]) + ',' + String(speeds[1]) + ',' + String(speeds[2]);

                COMMS_UART.println(command);

                // AX0Speed = abs(args[1].toFloat()) * 10;
                if (canData[0] < 0)
                {
                    sd.setDirection(TURNRIGHT);
                    AX0En = true;
                }
                else if (canData[0] > 0)
                {
                    sd.setDirection(TURNLEFT);
                    AX0En = true;
                }
                else
                {
                    AX0En = false;
                }
                lastCtrlCmd = millis();
            }
        }
    }


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

        String prevCommand;

        #ifdef DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.print("| Main MCU Command Recieved: ");
            Serial.println(input);
        #endif
        

        //--------//
        //  Misc  //
        //--------//
        /**/ if (command == "ping") {
            Serial.println("pong");
        }

        else if (command == "time") {
            Serial.println(millis());
        }
        // Refers to the Built In LED, not LED strip
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
        // TODO: Need to figure out how to output encoder values
        // TODO Need to add voltage, current and temp of the motors
        else if (args[0] == "data") // Send data out
        {

            if(args[1] == "sendEnc") // data
            {
                // outputEncoders();

            }
        }

        else if (args[0] == "can_relay_tovic")
        {
            vicCAN.relayFromSerial(args);
            Serial.println("Got Relay Command");
        }

        //------------//
        //  Physical  //
        //------------//

        else if (args[0] == "ctrl") // manual control, equivical to a ctrl command
        {
            float speeds[3];
            speeds[0] = args[2].toFloat() * 0.1;
            speeds[1] = args[3].toFloat() * 0.1;
            speeds[2] = args[4].toFloat() * 0.1;
            String command = "ctrl," + String(speeds[0]) + ',' + String(speeds[1]) + ',' + String(speeds[2]);

            #ifdef DEBUG
                    Serial.println("|------------------------------------------------------|");
                    Serial.println("| Main MCU Serial ctrl cmd recieved                    |");
            #endif

            COMMS_UART.println(command);

            // AX0Speed = abs(args[1].toFloat()) * 10;
            if (args[1].toFloat() < 0)
            {
                sd.setDirection(TURNRIGHT);
            }
            else
            {
                sd.setDirection(TURNLEFT);
            }
            AX0En = true;
            lastCtrlCmd = millis();
        }

        else if (args[0] == "IKA") // Set the target angle for IK
        {

            #ifdef DEBUG
                    Serial.println("|------------------------------------------------------|");
                    Serial.println("| Serial IK Angle cmd recieved                         |");
            #endif

            for (int i = 1; i <= MOTOR_AMOUNT; i++) 
            {
                AxisSetPosition[i-1] = args[i].toFloat();
            }
            lastCtrlCmd = millis();
        }

        else if (args[0] == "IKT") // Set the speed for each controller based on the given time
        {   
            
            #ifdef DEBUG
                    Serial.println("|------------------------------------------------------|");
                    Serial.println("| Serial IK Time cmd recieved                          |");
            #endif

            findSpeedandTime(args[1].toFloat());
            lastCtrlCmd = millis();
        }
    }

    // Relay data from the motor controller back over USB
    if (COMMS_UART.available())
    {
        String input = COMMS_UART.readStringUntil('\n');
        input.trim();

        #ifdef DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.print("| From Motor MCU Recieved: ");
            Serial.println(input);
        #endif
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
  if(millis() - lastCtrlCmd > 2000)// if no control commands are received for 2 seconds
  {
    lastCtrlCmd = millis();// just update the var so this only runs every 2 seconds.
    AX0En = 0;
    COMMS_UART.println("ctrl,0,0,0");

    #ifdef DEBUG
        Serial.println("|------------------------------------------------------|");
        Serial.println("|********************SAFETY TIMEOUT********************|");
    #else
        Serial.println("No Control / Safety Timeout");
    #endif
  }
}


// TODO: Needs to be complete- needs to get time to target and target angles per joint- how fast does the joint need to move
void findSpeedandTime(int time)               // Based on how long it will take for axis 0 to get to target location
{
    double setSpeed[MOTOR_AMOUNT];
    // Figure out the degrees per second
    for (int i = 0; i < MOTOR_AMOUNT; i++)
    {
        setSpeed[i] = abs(AxisPosition[i] - AxisSetPosition[i])/time;
    }

    convertToDutyCycleA0(setSpeed[0], 7.0625);
    convertToDutyCycle(setSpeed[1], 5000);
    convertToDutyCycle(setSpeed[2], 3750);
    convertToDutyCycle(setSpeed[3], 2500);

    // Send the ctrl, speed, speed, speed command here
    COMMS_UART.printf("ctrl,%i,%i,%i",setSpeed[1],setSpeed[2],setSpeed[3]);
}

// Pass by reference because it's easier
void convertToDutyCycle(double& dpsSpeed, float gearRatio)
{
    dpsSpeed = (dpsSpeed*gearRatio)/11000; // Retarded solution, should be changed
}

void convertToDutyCycleA0(double& dpsSpeed, float gearRatio)
{
    // Steps per millisecond
    dpsSpeed = (dpsSpeed*gearRatio)*7.63/1000;//0.05388
    // convert to period
    dpsSpeed = 1/dpsSpeed;
    #ifdef DEBUG
        Serial.println("|------------------------------------------------------|");
        Serial.print("| AX0 Speed Set To: ");
        Serial.println(dpsSpeed);
    #endif
    
    if (dpsSpeed < 0)
    {
        sd.setDirection(TURNRIGHT);
    }
    else
    {
        sd.setDirection(TURNLEFT);
    }
    AX0Speed = dpsSpeed;
    AX0En = true;
}

void updateMotorState()
{
    for (int i = 0; i <= MOTOR_AMOUNT; i++)
    {
        if(!AxisComplete[i])
        {
            if(abs(AxisPosition[i] - AxisSetPosition[i]) < 1)
            {
                // motorList[i-1]->stop();
                if (!i)
                {
                    AX0En = false;
                }
                else
                {
                COMMS_UART.printf("stop,%i",i);
                }
                
                AxisComplete[i] = true;
            }
        }
    } 
}

