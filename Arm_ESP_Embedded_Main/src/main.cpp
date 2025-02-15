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
void convertToDutyCycle(float& dpsSpeed, float gearRatio);
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
    sd.setStepMode(HPSDStepMode::MicroStep32);

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

    if((millis()-lastMotorStep)>=AX0Speed)
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


    //-------------//
    //  CAN Input  //
    //-------------//

    if(vicCAN.readCan()) {
        const uint8_t commandID = vicCAN.getCmdId();
        static std::vector<double> canData;
        vicCAN.parseData(canData);

        Serial.print("VicCAN: ");
        Serial.print(commandID);
        Serial.print("; ");
        if (canData.size() > 0) {
            for (const double& data : canData) {
                Serial.print(data);
                Serial.print(", ");
            }
        }
        Serial.println();


        // Misc

        /**/ if (commandID == CMD_PING) {
            vicCAN.respond(1);  // "pong"
            Serial.println("Received ping over CAN");
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
                    COMMS_UART.println("Got IK Angle,");
                #endif
                for (int i = 0; i <= MOTOR_AMOUNT; i++) 
                {
                    AxisSetPosition[i] = canData[i];
                }
            }
        }
        else if (commandID == CMD_ARM_IK_TTG) {
            if (canData.size() == 1) {
                #ifdef DEBUG
                    COMMS_UART.println("Got TTG Time,");
                #endif
                findSpeedandTime(canData[0]);
            }
        }
        else if (commandID == CMD_ARM_MANUAL) {
            if (canData.size() == 4) {

                String command = "ctrl," + String(canData[1]) + ',' + String(canData[2]) + ',' + String(canData[3]);
                COMMS_UART.println(command);
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

        //------------//
        //  Physical  //
        //------------//

        else if (args[0] == "ctrl") // manual control, equivical to a ctrl command
        {
            COMMS_UART.println(command);
        }

        else if (args[0] == "IKA") // Set the target angle for IK
        {
            for (int i = 1; i <= MOTOR_AMOUNT; i++) 
            {
                AxisSetPosition[i-1] = args[i].toInt();
            }
        }

        else if (args[0] == "IKT") // Set the speed for each controller based on the given time
        {    
            findSpeedandTime(args[1].toInt());
        }
    }

    // Relay data from the motor controller back over USB
    if (COMMS_UART.available())
    {
        String input = COMMS_UART.readStringUntil('\n');
        input.trim();
        Serial.println(input);
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
  if(millis() - lastCtrlCmd > 2000)//if no control commands are received for 2 seconds
  {
    // lastCtrlCmd = millis();//just update the var so this only runs every 2 seconds.

    COMMS_UART.println("ctrl,0,0,0");
    Serial.println("No Control / Safety Timeout");
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

    // Convert from dps to duty cycle
    // Stepper moder doesn't need this
    convertToDutyCycleA0(setSpeed[0], 7.0625);
    convertToDutyCycle(setSpeed[1], 5000);
    convertToDutyCycle(setSpeed[2], 3750);
    convertToDutyCycle(setSpeed[3], 2500);

    // Send the ctrl, speed, speed, speed command here
    COMMS_UART.printf("ctrl,%i,%i,%i",setSpeed[1],setSpeed[2],setSpeed[3]);

    // Add stepper speed

}

// Pass by reference because it's easier
void convertToDutyCycle(double& dpsSpeed, float gearRatio)
{
    dpsSpeed = (dpsSpeed*gearRatio)/11000; // Retarded solution, should be changed
}

void convertToDutyCycleA0(double& dpsSpeed, float gearRatio)
{
    // Steps per millisecond
    dpsSpeed = (dpsSpeed*gearRatio)*7.63*1000; // Retarded solution, should be changed
    // convert to period
    dpsSpeed = 1/dpsSpeed;
    #ifdef DEBUG
        Serial.printf("AX0 Period Set: %d", dpsSpeed);
    #endif
    AX0Speed = dpsSpeed;
}

void updateMotorState()
{
    for (int i = 1; i <= MOTOR_AMOUNT; i++)
    {
        if(!AxisComplete[i-1])
        {
            if(abs(AxisPosition[i-1] - AxisSetPosition[i-1]) < 1) //TODO: Validate 1 degree precision
            {
                // motorList[i-1]->stop();
                if (!i)
                {
                    // Stop the stepper motor
                }
                else
                {
                COMMS_UART.printf("stop,%i",i);
                }
                
                AxisComplete[i-1] = true;
            }
        }
    } 
}

