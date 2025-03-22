/**
 * @file main.cpp
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
#include "AS5047P.h"

// Our own resources

#include "AstraMisc.h"
#include "AstraVicCAN.h"
#include "project/ARM.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

// #define ARM_DEBUG

#define SPI_BUS_SPEED 1000000 // 1MHz

#define ax0_encoder_offset 140
#define ax1_encoder_offset 55
#define ax2_encoder_offset 352
#define ax3_encoder_offset 55


//---------------------//
//  Component classes  //
//---------------------//

HighPowerStepperDriver sd;

AS5047P ax0_encoder(ENCODER_AXIS0_PIN, SPI_BUS_SPEED); 
AS5047P ax1_encoder(ENCODER_AXIS1_PIN, SPI_BUS_SPEED); 
AS5047P ax2_encoder(ENCODER_AXIS2_PIN, SPI_BUS_SPEED); 
AS5047P ax3_encoder(ENCODER_AXIS3_PIN, SPI_BUS_SPEED); 
auto errorInfo = AS5047P_Types::ERROR_t();


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

const uint16_t StepPeriodUs = 2000;

unsigned long lastFeedback = 0;
unsigned long lastMotorStep = 0;
unsigned long lastCtrlCmd = 0;
unsigned long lastEncoderRead = 0;
unsigned long lastEncoderOutput = 0;
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
void readEncoders();
int adjustEncoderValue(int encoderValue, int offset);

void safety_timeout();
void findSpeedandTime(int time);
void convertToDutyCycle(double& dpsSpeed, float gearRatio);
void convertToDutyCycleA0(double& dpsSpeed, float gearRatio);
void updateMotorState();


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

    if(ESP32Can.begin(TWAI_SPEED_1000KBPS, 13, 12))
        Serial.println("CAN bus started!");
    else
        Serial.println("CAN bus failed!");

    SPI.begin();


    //-----------------//
    //  Stepper Motor  //
    //-----------------//

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
    sd.setStepMode(HPSDStepMode::MicroStep2);

    // Enable the motor outputs.
    sd.enableDriver();

    //-----------------//
    //  Encoder Setup  //
    //-----------------//

    // initialize the AS5047P sensor and hold if sensor can't be initialized.
    if(!ax0_encoder.initSPI()) {
        Serial.println(F("Axis0 Encoder: Failed"));
    } else {
        Serial.println("Axis0 Encoder: Success");
    }

    if(!ax1_encoder.initSPI()) {
        Serial.println(F("Axis1 Encoder: Failed"));
    } else {
        Serial.println("Axis1 Encoder: Success");
    }
  
    if(!ax2_encoder.initSPI()) {
        Serial.println(F("Axis2 Encoder: Failed"));
    } else {
        Serial.println("Axis2 Encoder: Success");
    }
  
    if(!ax3_encoder.initSPI()) {
        Serial.println(F("Axis3 Encoder: Failed"));
    } else {
        Serial.println("Axis3 Encoder: Success");
    }
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

    if ((millis() - lastMotorStep >= 1) && AX0En)
    {
        lastMotorStep = millis();
        sd.step();
    }

    if (millis() - lastFeedback >= 2000)
    {
        
        lastFeedback = millis();
    }

    if (millis() - lastEncoderRead >= 250)
    {
        lastEncoderRead = millis();
        readEncoders();
    }

    // Safety timeout if no ctrl command for 2 seconds
    if(millis() - lastCtrlCmd > 2000)
    {
        lastCtrlCmd = millis();
        AX0En = 0;
        COMMS_UART.println("ctrl,0,0,0");

#ifdef ARM_DEBUG
        Serial.println("|------------------------------------------------------|");
        Serial.println("|********************SAFETY TIMEOUT********************|");
#else
        Serial.println("No Control / Safety Timeout");
#endif
    }

    outputEncoders();


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

        if (commandID == CMD_PING) {
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
#ifdef ARM_DEBUG
                Serial.println("|------------------------------------------------------|");
                Serial.println("| VicCan IK Angle cmd recieved                         |");
#endif
                for (int i = 0; i < MOTOR_AMOUNT; i++) 
                {
                    AxisSetPosition[i] = canData[i];
                }
                lastCtrlCmd = millis();
                moveToPossition();
            }
        }
        else if (commandID == CMD_ARM_IK_TTG) {
            if (canData.size() == 1) {
#ifdef ARM_DEBUG
                Serial.println("|------------------------------------------------------|");
                Serial.println("| VicCan IK Time cmd recieved                          |");
#endif
                findSpeedandTime(canData[0]);
            }
        }
        else if (commandID == CMD_ARM_MANUAL) {
            if (canData.size() == 4) {
                lastCtrlCmd = millis();
#ifdef ARM_DEBUG
                Serial.println("|------------------------------------------------------|");
                Serial.println("| VicCan ctrl cmd recieved                             |");
#endif

                float speeds[3];
                speeds[0] = canData[1] * 0.75;
                speeds[1] = canData[2] * 0.50;
                speeds[2] = canData[3] * 0.50;

                COMMS_UART.printf("ctrl,%f,%f,%f\n", speeds[0], speeds[1], speeds[2]);

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

#ifdef ARM_DEBUG
        Serial.println("|------------------------------------------------------|");
        Serial.print("| Main MCU Command Recieved: ");
        Serial.println(input);
#endif
        

        //--------//
        //  Misc  //
        //--------//
        if (command == "ping") {
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

        else if (args[0] == "can_relay_mode") {
            if (args[1] == "on") {
                vicCAN.relayOn();
            } else if (args[1] == "off") {
                vicCAN.relayOff();
            }
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

#ifdef ARM_DEBUG
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

#ifdef ARM_DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.println("| Serial IK Angle cmd recieved                         |");
#endif

            for (int i = 0; i < MOTOR_AMOUNT; i++) 
            {
                AxisSetPosition[i] = args[i + 1].toFloat();
            }
            lastCtrlCmd = millis();
            moveToPossition();
        }

        else if (args[0] == "IKT") // Set the speed for each controller based on the given time
        {   
            
#ifdef ARM_DEBUG
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

#ifdef ARM_DEBUG
        Serial.println("|------------------------------------------------------|");
        Serial.print("| From Motor MCU Recieved: ");
#endif
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

int adjustEncoderValue(int encoderValue, int offset) {
    int adjustedValue = encoderValue - offset;
    if (adjustedValue < 0) {
        adjustedValue += 360; // Wrap around at 360 degrees
    }
    return adjustedValue % 360; // Ensure the value is within 0-359 range
}

void outputEncoders()
{
    if((millis()-lastEncoderOutput>=500))
    {
        lastEncoderOutput = millis();
        char buffer[50];
        sprintf(buffer, "AX0: %3d\tAX1: %3d\tAX2: %3d\tAX3: %3d", AxisPosition[0], AxisPosition[1], AxisPosition[2], AxisPosition[3]);
        Serial.println(buffer);
    }
}


void readEncoders()
{
    AxisPosition[0] = adjustEncoderValue(round(ax0_encoder.readAngleDegree(true, &errorInfo, true, true, true)), ax0_encoder_offset);
    AxisPosition[1] = adjustEncoderValue(round(ax1_encoder.readAngleDegree(true, &errorInfo, true, true, true)), ax1_encoder_offset);
    AxisPosition[2] = adjustEncoderValue(round(ax2_encoder.readAngleDegree(true, &errorInfo, true, true, true)), ax2_encoder_offset);
    AxisPosition[3] = adjustEncoderValue(round(ax3_encoder.readAngleDegree(true, &errorInfo, true, true, true)), ax3_encoder_offset);
}


// TODO: Needs to be complete- needs to get time to target and target angles per joint- how fast does the joint need to move
void findSpeedandTime(int time)               // Based on how long it will take for axis 0 to get to target location
{
    double setSpeed[MOTOR_AMOUNT];
    // Figure out the degrees per second
    readEncoders();
    for (int i = 0; i < MOTOR_AMOUNT; i++)
    {
        setSpeed[i] = abs(AxisPosition[i] - AxisSetPosition[i])/time;
    }

    convertToDutyCycleA0(setSpeed[0], 7.0625);
    convertToDutyCycle(setSpeed[1], 5000);
    convertToDutyCycle(setSpeed[2], 3750);
    convertToDutyCycle(setSpeed[3], 2500);

    // Send the ctrl, speed, speed, speed command here
    COMMS_UART.printf("ctrl,%f,%f,%f\n", setSpeed[1], setSpeed[2], setSpeed[3]);
}

int findRotationDirection(float current_direction, float target_direction)
{
    if (current_direction <= target_direction)
    {
        return 1; 
    }
    else
    {
        return 0;
    }
}

void moveToPossition()               // Based on how long it will take for axis 0 to get to target location
{
    double setSpeed[MOTOR_AMOUNT];
    // Figure out the degrees per second
    readEncoders();
    for (int i = 0; i < MOTOR_AMOUNT; i++)
    {
        if (findRotationDirection(AxisPosition[i],AxisSetPosition[i]))
        {
            setSpeed[i] = 0.1;
        }
        else
        {
            setSpeed[i] = -0.1;
        }
    }

    // Send the ctrl, speed, speed, speed command here
    COMMS_UART.printf("ctrl,%f,%f,%f\n", setSpeed[1], setSpeed[2], setSpeed[3]);
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
#ifdef ARM_DEBUG
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
    for (int i = 0; i < MOTOR_AMOUNT; i++)
    {
        if(!AxisComplete[i] && abs(AxisPosition[i] - AxisSetPosition[i]) < 1)
        {
            // motorList[i-1]->stop();
            if (i == 0)
            {
                AX0En = false;
            }
            else
            {
                COMMS_UART.printf("stop,%i\n",i);
            }
            
            AxisComplete[i] = true;
        }
    } 
}
