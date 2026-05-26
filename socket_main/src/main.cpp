/**
 * @file main.cpp
 * @author Charles Marmann (cmm0077@uah.edu)
 * @author Jack Schumacher (js0342@uah.edu)
 * @author Tristan McGinnis (tlm0047@uah.edu)
 * @brief Arm Embedded Main MCU
 *
 */

//------------//
//  Includes  //
//------------//

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <SPI.h>

#include <cmath>

#include "AS5047P.h"

// Our own resources

#include "ArmMainMCU.h"
#include "AstraArm.h"
#include "AstraMisc.h"
#include "AstraMotors.h"
#include "AstraVicCAN.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

// #define ARM_DEBUG

#define SPI_BUS_SPEED 1000000  // 1MHz



// REV Motor IDs
#define MOTOR_ID_0 0
#define MOTOR_ID_1 1
#define MOTOR_ID_2 2
#define MOTOR_ID_3 3
#define MOTOR_AMOUNT 4

//---------------------//
//  Component classes  //
//---------------------//

AS5047P ax0_encoder(ENCODER_AXIS0_PIN, SPI_BUS_SPEED);
AS5047P ax1_encoder(ENCODER_AXIS1_PIN, SPI_BUS_SPEED);
AS5047P ax2_encoder(ENCODER_AXIS2_PIN, SPI_BUS_SPEED);
AS5047P ax3_encoder(ENCODER_AXIS3_PIN, SPI_BUS_SPEED);

// ArmJoint(AS5047P* setEncoder, float setZeroAngle, float setMinAngle, float setMaxAngle, int setGearRatio,
// bool setInverted);
// TODO: Update for new arm
ArmJoint axis0(&ax0_encoder, 179, -179, 135, 468);  // 64:1 gearbox, 16:117 small and big gears
ArmJoint axis1(&ax1_encoder, 55, -60, 90, 5000);
ArmJoint axis2(&ax2_encoder, 352, -115, 115, 3750);
ArmJoint axis3(&ax3_encoder, 7.5, -90, 110, 2500);
ArmJoint* joints[] = {&axis0, &axis1, &axis2, &axis3};

AstraArm arm(joints);


// AstraMotors(int setMotorID, bool setInverted, int setGearBox)
AstraMotors Motor1(MOTOR_ID_0, false);  // Front Left
AstraMotors Motor2(MOTOR_ID_1, false);  // Back Left
AstraMotors Motor3(MOTOR_ID_2, true);   // Front Right
AstraMotors Motor4(MOTOR_ID_3, true);   // Back Right

AstraMotors* armMotors[4] = {&Motor1, &Motor2, &Motor3, &Motor4};


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

const uint16_t StepPeriodUs = 2000;  // Old??
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

Timer EncoderFeedback;
Timer VoltageFeedback;
Timer CtrlCmdTimeout;
Timer IKUpdate;
Timer HeartBeat;
Timer Blink;

//--------------//
//  Prototypes  //
//--------------//

void Stop();

bool trigger(Timer& timer) {
    bool isTriggered = millis() - timer.lastMillis >= timer.interval;
    timer.lastMillis = millis();
    return isTriggered;
};

bool spiInit(AS5047P* encoder, int8_t spi_clk, int8_t spi_miso, int8_t spi_mosi, int8_t spi_cs);

void heartbeatTask(void* pvParameters) {
    while (true) {
        if (trigger(HeartBeat)) {
            for (size_t i = 1; i <= 4; i++) {
                CAN_sendHeartbeat(i);
            }
        }
    }
}

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
void setup() {
    //--------//
    //  Pins  //
    //--------//

    bool encoderFailure = false;

    bool canFailure = false;

    //-----------//
    //  MCU LED  //
    //-----------//

    pixel.begin();
    pixel.clear();
    pixel.setPixelColor(0, 244, 30, 10);
    pixel.show();

    //----------//
    //  Timers  //
    //----------//

    // Timer intervals in milliseconds

    EncoderFeedback.interval = 100;
    VoltageFeedback.interval = 1000;
    CtrlCmdTimeout.interval = 2000;
    IKUpdate.interval = 50;
    HeartBeat.interval = 10;
    Blink.interval = 1000;

    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);
    // COMMS_UART.begin(COMMS_UART_BAUD);

    if (ESP32Can.begin(TWAI_SPEED_1000KBPS, CAN_TX, CAN_RX))
        Serial.println("CAN bus started!");
    else {
        Serial.println("CAN bus failed!");
        canFailure = true;
    }

    SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI, ENCODER_AXIS0_PIN);

    //-----------------//
    //  Encoder Setup  //
    //-----------------//

    // initialize the AS5047P sensor and hold if sensor can't be initialized.
    if (!spiInit(&ax0_encoder, SPI_CLK, SPI_MISO, SPI_MOSI, ENCODER_AXIS0_PIN)) {
        Serial.println(F("Axis0 Encoder: Failed"));
        encoderFailure = true;
    } else {
        Serial.println("Axis0 Encoder: Success");
    }

    if (!spiInit(&ax1_encoder, SPI_CLK, SPI_MISO, SPI_MOSI, ENCODER_AXIS1_PIN)) {
        Serial.println(F("Axis1 Encoder: Failed"));
        encoderFailure = true;
    } else {
        Serial.println("Axis1 Encoder: Success");
    }

    if (!spiInit(&ax2_encoder, SPI_CLK, SPI_MISO, SPI_MOSI, ENCODER_AXIS2_PIN)) {
        Serial.println(F("Axis2 Encoder: Failed"));
        encoderFailure = true;
    } else {
        Serial.println("Axis2 Encoder: Success");
    }

    if (!spiInit(&ax3_encoder, SPI_CLK, SPI_MISO, SPI_MOSI, ENCODER_AXIS3_PIN)) {
        Serial.println(F("Axis3 Encoder: Failed"));
        encoderFailure = true;
    } else {
        Serial.println("Axis3 Encoder: Success");
    }

    xTaskCreatePinnedToCore(heartbeatTask, "heartbeat", 1000, NULL, 0, NULL, 0);

    delay(1000);

    Serial.println("Setup is complete");
    if (encoderFailure) {
        pixel.setPixelColor(0, 174, 5, 252);  // Purple
        pixel.show();
    } else if (encoderFailure && canFailure) {
        pixel.setPixelColor(0, 1, 230, 30);  // Orange
        pixel.show();
    } else if (canFailure) {
        pixel.setPixelColor(0, 5, 30, 252);  // Blue
        pixel.show();
    } else {
        pixel.setPixelColor(0, 1, 204, 23);
        pixel.show();
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
    if (trigger(Blink)) {
        ledState = !ledState;
        pixel.setBrightness(255 * (uint8_t)ledState);
    }
#endif

    if (trigger(VoltageFeedback)) {
        float vBatt = convertADC(analogRead(PIN_VDIV_BATT), 10, 2.21);
        float v12 = convertADC(analogRead(PIN_VDIV_12V), 10, 3.32);
        float v5 = convertADC(analogRead(PIN_VDIV_5V), 10, 10);
        float v33 = convertADC(analogRead(PIN_VDIV_3V3), 10, 10);

        vicCAN.send(CMD_POWER_VOLTAGE, vBatt * 100, v12 * 100, v5 * 100, v33 * 100);
    }

    if (trigger(EncoderFeedback)) {
        vicCAN.send(CMD_ARM_ENCODER_ANGLES, axis0.lastEffectiveAngle * 10, axis1.lastEffectiveAngle * 10,
                    axis2.lastEffectiveAngle * 10, axis3.lastEffectiveAngle * 10);
#ifdef DEBUG
        Serial.println("|------------------------------------------------------|");
        Serial.println("|********************Encoder Feedback******************|");
        Serial.printf("Axis0: %f\tAxis1: %f\tAxis2: %f\tAxis3: %f\n", axis0.lastEffectiveAngle,
                      axis1.lastEffectiveAngle, axis2.lastEffectiveAngle, axis3.lastEffectiveAngle);
#endif
    }

    // Safety timeout if no ctrl command for 2 seconds
    if (trigger(CtrlCmdTimeout)) {
        // arm.stop();

#ifdef DEBUG
        Serial.println("|------------------------------------------------------|");
        Serial.println("|********************SAFETY TIMEOUT********************|");
#else
        Serial.println("Safety timeout");
#endif
    }

    if (trigger(IKUpdate)) {
        arm.updateIKMotion();
    }

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

    CanFrame receivedFame;
    bool isRevCan;
    if (vicCAN.readCan(&isRevCan, &receivedFame)) {
        const uint8_t commandID = vicCAN.getCmdId();
        static std::vector<double> canData;
        vicCAN.parseData(canData);

#ifdef DEBUG
        Serial.println("|------------------------------------------------------|");
        Serial.println("|***************MCU VicCAN Received Data***************|");
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
        if (!isRevCan) {
            if (commandID == CMD_PING) {
                vicCAN.respond(1);  // "pong"
                Serial.println("Received ping over CAN");
            } else if (commandID == CMD_B_LED) {
                if (canData.size() == 1) {
                    if (canData[0] == 0)
                        pixel.setBrightness(0);
                    if (canData[0] == 1)
                        pixel.setBrightness(255);
                }
            }
        } else {
            // REV
            if (commandID == CMD_REV_STOP) {
                Stop();
            } else if (commandID == CMD_REV_IDENTIFY) {
                if (canData.size() == 1) {
                    CAN_identifySparkMax(canData[0]);
#ifdef DEBUG

                    Serial.print("rev_id,");
                    Serial.println(canData[0]);
#endif
                }
            } else if (commandID == CMD_REV_IDLE_MODE) {
                if (canData.size() == 1 && (canData[0] == 0 || canData[0] == 1)) {
                    for (uint8_t i = 0; i < 4; i++) {
                        armMotors[i]->setBrake(canData[0]);
                    }
                }
#ifdef DEBUG
                else {
                    Serial.println("Improper REV Idle Mode Command Received");
                }
#endif
            } else if (commandID == CMD_ARM_IK_CTRL) {
                if (canData.size() == 4) {
#ifdef DEBUG
                    Serial.println("|------------------------------------------------------|");
                    Serial.println("|***********VicCan IK Angle cmd received***************|");
#endif
                    CtrlCmdTimeout.lastMillis = millis();
                    float speeds[4] = {0};
                    speeds[0] = canData[0] == 0 ? 0 : canData[0] / 10.0;
                    speeds[1] = canData[1] == 0 ? 0 : canData[1] / 10.0;
                    speeds[2] = canData[2] == 0 ? 0 : canData[2] / 10.0;
                    speeds[3] = canData[3] == 0 ? 0 : canData[3] / 10.0;
                    arm.setTargetAngles(speeds[0], speeds[1], speeds[2], speeds[3]);
                }
#ifdef DEBUG
                else {
                    Serial.println("Incorrect IK Control Command Received");
                }
#endif
            } else if (commandID == CMD_ARM_IK_TTG) {
                if (canData.size() == 1) {
#ifdef DEBUG
                    Serial.println("|------------------------------------------------------|");
                    Serial.println("|VicCan IK Time cmd received                          |");
#endif
                    arm.setTTG(canData[0]);
                }
            } else if (commandID == CMD_ARM_MANUAL) {
                if (canData.size() == 4) {
#ifdef DEBUG
                    Serial.println("|------------------------------------------------------|");
                    Serial.println("|**********VicCan ctrl cmd received********************|");
#endif
                    CtrlCmdTimeout.lastMillis = millis();
                    float speeds[4] = {0};
                    for (int i = 0; i < 4; i++) {
                        speeds[i] = canData[i] * 0.75;
                    }
                    arm.runDuty(speeds);
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
        parseInput(input, args);        // Separate `input` by commas and place into args vector
        args[0].toLowerCase();          // Make command case-insensitive
        String command = args[0];       // To make processing code more readable

        String prevCommand;

#ifdef DEBUG
        Serial.println("|------------------------------------------------------|");
        Serial.print("| Main MCU Command Received: ");
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
        else if (args[0] == "data")  // Send data out
        {
            if (args[1] == "sendEnc")  // data
            {
                // outputEncoders();
            }
        }

        else if (args[0] == "can_relay_tovic") {
            vicCAN.relayFromSerial(args);
#ifdef DEBUG
            Serial.println("Got Relay Command");
#endif
        }

        else if (args[0] == "can_relay_mode") {
            if (args[1] == "on") {
                vicCAN.relayOn();
            } else if (args[1] == "off") {
                vicCAN.relayOff();
            }
        }

        else if (args[0] == "effangles") {
            Serial.printf("Axis0: %f\tAxis1: %f\tAxis2: %f\tAxis3: %f\n", axis0.lastEffectiveAngle,
                          axis1.lastEffectiveAngle, axis2.lastEffectiveAngle, axis3.lastEffectiveAngle);
        }

        else if (args[0] == "stop") {
            arm.stop();
        }

        //------------//
        //  Physical  //
        //------------//

        else if (args[0] == "ctrl")  // manual control, equivocal to a ctrl command
        {
#ifdef DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.println("| Main MCU Serial ctrl cmd received                    |");
#endif
            CtrlCmdTimeout.lastMillis = millis();
            // COMMS_UART.println(input);
        }

        else if (args[0] == "IKA")  // Set the target angle for IK
        {
#ifdef DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.println("| Serial IK Angle cmd recieved                         |");
#endif

            CtrlCmdTimeout.lastMillis = millis();
        }

        else if (args[0] == "IKT")  // Set the speed for each controller based on the given time
        {
#ifdef DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.println("| Serial IK Time cmd recieved                          |");
#endif

            CtrlCmdTimeout.lastMillis = millis();
        }
    }

    // Relay data from the motor controller back over USB
    //    if (COMMS_UART.available())
    //    {
    //        String input = COMMS_UART.readStringUntil('\n');
    //        input.trim();
    //        std::vector<String> args = {};  // Initialize empty vector to hold separated arguments
    //        parseInput(input, args);   // Separate `input` by commas and place into args vector
    //
    // #ifdef ARM_DEBUG
    //        Serial.println("|------------------------------------------------------|");
    //        Serial.print("| From Motor MCU Recieved: ");
    // #endif
    //        Serial.print("Motor MCU:\t");
    //        Serial.println(input);
    //
    //        if (checkArgs(args, 4) && args[0] == "motorstatus") {
    //            vicCAN.send(CMD_REVMOTOR_FEEDBACK, args[1].toInt(), args[2].toInt(), args[3].toInt(),
    //            args[4].toInt());
    //        }
    //    }
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

bool spiInit(AS5047P* encoder, int8_t spi_clk, int8_t spi_miso, int8_t spi_mosi, int8_t spi_cs) {
    SPI.begin(spi_clk, spi_miso, spi_mosi, spi_cs);
    return encoder->checkSPICon();
}

void Stop() {
    for (int i = 0; i < 4; i++) {
        armMotors[i]->stop();
    }
}
