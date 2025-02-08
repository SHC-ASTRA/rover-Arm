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
#include <FastLED.h>
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

// strip 1: 1-40
// strip 2: 41-82
// strip 3: 83-124
// strip 4: 125-166
// CCW: 1,2,3,4
#define NUM_LEDS 166


//---------------------//
//  Component classes  //
//---------------------//

// LED Strip
int led_rbg[3] = {0, 300, 0}; //When using multiple colors, use 255 max, when doing R/B/G use 800-900 for best brightness
int led_counter = 0;
CRGB leds[NUM_LEDS];

//Sensor declarations

AstraCAN Can0;


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

unsigned long clockTimer = 0;
unsigned long lastFeedback = 0;
unsigned long lastCtrlCmd = 0;

unsigned int goalTime;

bool AxisComplete    [] = {true,true,true,true};     // AxisXComplete    where x = 1..3
int  AxisSetPosition [] = {0,0,0,0};              // AxisXSetPosition ^^^
int  AxisPosition    [] = {0,0,0,0};              // AxisXPosition    ^^^


//--------------//
//  Prototypes  //
//--------------//

void outputEncoders();
void safety_timeout();
void updateMotorStatus();

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


    //-----------//
    //  Sensors  //
    //-----------//


    //--------------------//
    //  Misc. Components  //
    //--------------------//

    
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
            COMMS_UART.println("ctrl,stop");
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
        else if (commandID == CMD_REV_SET_DUTY) {
            if (canData.size() == 2) {
                COMMS_UART.print("ctrl,");
                COMMS_UART.print(canData[0]);
                COMMS_UART.print(",");
                COMMS_UART.println(canData[1]);
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

        else if (args[0] == "data") // Send data out
        {

            if(args[1] == "sendEnc") // data,sendGPS
            {
                // outputEncoders();
            }
        }

        //------------//
        //  Physical  //
        //------------//

        else if (args[0] == "ctrl") // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        {
            COMMS_UART.print(command);
            lastCtrlCmd = millis();
        }

        else if (args[0] == "Man") // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        {
            Serial1.println(command);

            int speed1 = args[1].toInt();
            int speed2 = args[2].toInt();
            int speed3 = args[3].toInt();
            int speed4 = args[4].toInt();

            COMMS_UART.printf("ctrl,%s,%s,%s,%s", speed1, speed2, speed3, speed4);

        }

        else if (args[0] == "IKA") // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        {
            Serial1.println(command);

            //COMMS_UART.printf("ctrl,%s,%s,%s",args[4],args[5],args[6]);

            for (int i = 1; i <= MOTOR_AMOUNT; i++) {
                AxisSetPosition[i-1] = args[i].toInt();
            }
        }

        else if (args[0] == "IKT") // Is looking for a command that looks like "ctrl,LeftY-Axis,RightY-Axis" where LY,RY are >-1 and <1
        {
            Serial1.println(command);

            // findSpeedandTime(args[1].toInt(),args[2].toInt(),args[3].toInt(),args[4].toInt());
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

void safety_timeout(){
  if(millis() - lastCtrlCmd > 2000)//if no control commands are received for 2 seconds
  {
    // lastCtrlCmd = millis();//just update the var so this only runs every 2 seconds.

    Serial1.println("ctrl,0,0,0");
    Serial.println("No Control / Safety Timeout");
  }
}

void findSpeedandTime(int ax0, int ax1, int ax2, int ax3)               // Based on how long it will take for axis 0 to get to target location
{
    
    // find distance from ax current and ax desired
    // d/v = t
    // use for finding the speeds for all other joints
    // v = d/t
}


// Prints the output of the BNO in one line
String outputBno()
{
    float bnoData2[7];
    String output;
    //sprintf(output,"%f,%f,%f,%f,%f,%f,%f",bnoData2[0],bnoData2[1],bnoData2[2],bnoData2[3],bnoData2[4],bnoData2[5],bnoData2[6]);
    
    return output;
}


void updateMotorStatus()
{
    for (int i = 1; i <= MOTOR_AMOUNT; i++)
    {
        if(!AxisComplete[i-1])
        {
            if(abs(AxisPosition[i-1] - AxisSetPosition[i-1]) < 10)  // 10 degree percision !!NOT GOOD NEEDS TO BE CHANGED!!
            {
                // motorList[i-1]->stop();
                COMMS_UART.printf("stop,%i",i);
                
                AxisComplete[i-1] = true;
            }
        }
    }
    
}

void setLED(int r_val, int b_val, int g_val)
{
    for(int i = 0; i < NUM_LEDS; ++i)
    {
      leds[i] = CRGB(r_val, b_val, g_val);
      FastLED.show();
      delay(10);
    }
}
