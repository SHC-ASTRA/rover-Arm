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
#define MOTOR_ID_0 4
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

// TODO: Check for reversed motors

// AstraMotors(int setMotorID, bool setInverted, int setGearBox)
AstraMotors Motor0(MOTOR_ID_0, false);  // Axis 0
AstraMotors Motor1(MOTOR_ID_1, true);   // Axis 1
AstraMotors Motor2(MOTOR_ID_2, false);  // Axis 2
AstraMotors Motor3(MOTOR_ID_3, false);  // Axis 3

AstraMotors* armMotors[4] = {&Motor0, &Motor1, &Motor2, &Motor3};

// ArmJoint(AS5047P* setEncoder, float setZeroAngle, float setMinAngle, float setMaxAngle, int setGearRatio,
// bool setInverted);
// TODO: Update for new arm
ArmJoint axis0(&Motor0, &ax0_encoder, 179, -179, 135, 468);  // 64:1 gearbox, 16:117 small and big gears
ArmJoint axis1(&Motor1, &ax1_encoder, 55, -60, 90, 5000);
ArmJoint axis2(&Motor2, &ax2_encoder, 352, -115, 115, 3750);
ArmJoint axis3(&Motor3, &ax3_encoder, 7.5, -90, 110, 2500);
ArmJoint* joints[] = {&axis0, &axis1, &axis2, &axis3};

AstraArm arm(joints);


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

const uint16_t StepPeriodUs = 2000;  // Old??

Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
uint32_t neoPixelColor;

Timer EncoderFeedback;
Timer VoltageFeedback;
Timer CtrlCmdTimeout;
Timer IKUpdate;
Timer HeartBeat;
Timer Blink;
Timer revFeedback;
Timer versionFeedback;

//--------------//
//  Prototypes  //
//--------------//

void Stop();

bool spiInit(AS5047P* encoder, int8_t spi_clk, int8_t spi_miso, int8_t spi_mosi, int8_t spi_cs);

inline bool trigger(Timer& timer) {
    bool isTriggered = millis() - timer.lastMillis >= timer.interval;
    if (isTriggered) {
        timer.lastMillis = millis();
    }
    return isTriggered;
};

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
    Blink.interval = 800;
    revFeedback.interval = 500;
    versionFeedback.interval = 5000;

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

    neoPixelColor = pixel.getPixelColor(0);
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
        pixel.setPixelColor(0, neoPixelColor * ledState);
        pixel.show();
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
#ifndef DEBUG
        arm.stop();
#endif

#ifdef DEBUG
        Serial.println("|------------------------------------------------------|");
        Serial.println("|********************SAFETY TIMEOUT********************|");
#else
        Serial.println("Safety timeout");
#endif
    }

    // if (trigger(IKUpdate)) {
    //     arm.updateIKMotion();
    // }

    // Motor status debug printout
    if (trigger(revFeedback)) {

        for (int i = 0; i < 4; i++) {
            if (millis() - armMotors[i]->status1.timestamp < 500) {
                vicCAN.send(CMD_REVMOTOR_FEEDBACK, armMotors[i]->getID(),
                            armMotors[i]->status1.motorTemperature * 10,
                            armMotors[i]->status1.busVoltage * 10, armMotors[i]->status1.outputCurrent * 10);
            }
            if (millis() - armMotors[i]->status1.timestamp < 500 &&
                millis() - armMotors[i]->status2.timestamp < 500) {
                vicCAN.send(58, armMotors[i]->getID(), armMotors[i]->status2.sensorPosition,
                            armMotors[i]->status1.sensorVelocity);
            }
        }
    }

    if (trigger(versionFeedback)) {
        SEND_VERSION_INFO
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

    CanFrame receivedFrame;
    bool isRevCan;
    if (vicCAN.readCan(&isRevCan, &receivedFrame)) {
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

        // REV
        else if (commandID == CMD_REV_STOP) {
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

        // } else if (commandID == CMD_REV_SET_DUTY) {
        //     if (canData.size() == 4) {
        //         for (int i = 0; i < 4; i++) {
        //             armMotors[i]->sendDuty(canData[i]);
        //         }
        //     }

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
#ifdef DEBUG
            else {
                Serial.println("Improper CMD_ARM_MANUAL command data length");
                Serial.println("Perhaps this was meant for digit?");
            }
#endif
        }
    } else if (isRevCan) {                                         // REV Motor Feedback
        uint8_t deviceId = receivedFrame.identifier & 0x3F;        // [5:0]
        uint32_t apiId = (receivedFrame.identifier >> 6) & 0x3FF;  // [15:6]

#if defined(DEBUG_STATUS)
        // Log message if it seems interesting
        if (apiId == 0x99 || (apiId & 0x60) == 0x60 || (apiId & 0x300) == 0x300) {
            printREVFrame(receivedFrame);
        }
#endif

        if ((apiId & 0x60) == 0x60) {  // Periodic status
            for (int i = 0; i < 4; i++) {
                if (deviceId == armMotors[i]->getID()) {
                    armMotors[i]->parseStatus(apiId, receivedFrame.data);
                    break;
                }
            }
        } else if ((apiId & 0x300) == 0x300) {  // Parameter
            printREVParameter(receivedFrame);
#ifdef DEBUG
            Serial.print("From frame: ");
            printREVFrame(receivedFrame);
#endif
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
        // TODO Need to add voltage, current and temp of the motors
        else if (args[0] == "data")  // Send data out
        {
            if (args[1] == "sendEnc")  // data
            {
                Serial.printf("Axis0: %f\tAxis1: %f\tAxis2: %f\tAxis3: %f\n", axis0.lastEncoderAngle,
                              axis1.lastEncoderAngle, axis2.lastEncoderAngle, axis3.lastEncoderAngle);
            }
        }

        else if (args[0] == "motor_feedback") {
            if (args[1] == "sendVoltage") {
            } else if (args[1] == "sendCurrent") {
            } else if (args[1] == "sendTemp") {
            }
        }

        else if (args[0] == "can_relay_tovic") {
#ifdef DEBUG
            Serial.println("Got Relay Command");
#endif
            vicCAN.relayFromSerial(args);
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

        else if (args[0] == "motor_duty_cycle") {
#ifdef DEBUG
            Serial.printf("Setting Motor%c to %c duty cycle\n", &args[1], &args[2]);
#endif
            armMotors[args[1].toInt()]->sendDuty(args[2].toFloat());

        }

        else if (args[0] == "motor_speed") {
#ifdef DEBUG
            Serial.printf("Setting Motor1 to %f, Motor2 to %f, Motor3 to %f, Motor4 to %f,  ", &args[1],
                          &args[2], &args[3], &args[4]);
#endif
            armMotors[0]->sendSpeed(args[1].toFloat());
            armMotors[1]->sendSpeed(args[2].toFloat());
            armMotors[2]->sendSpeed(args[3].toFloat());
            armMotors[3]->sendSpeed(args[4].toFloat());
        }

        else if (args[0] == "stop") {
            arm.stop();
        }

        else if (args[0] == "digit_linear_ac") {
#ifdef DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.println("|*************Sent Digit Linear AC Command*************|");
#endif
            vicCAN.send(CMD_DIGIT_LINAC_CTRL, args[1].toDouble());
        }

        else if (args[0] == "digit_wrist") {
#ifdef DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.println("|***************Sent Digit Wrist Command***************|");
#endif
            vicCAN.send(CMD_ARM_MANUAL, args[1].toFloat(), args[2].toFloat());
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
            for (size_t i = 0; i < 4; i++) {
                armMotors[i]->turnByDeg((args[i + 1]).toFloat());
            }
        }

        else if (args[0] == "IKA")  // Set the target angle for IK
        {
#ifdef DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.println("| Serial IK Angle cmd received                         |");
#endif

            CtrlCmdTimeout.lastMillis = millis();
        }

        else if (args[0] == "IKT")  // Set the speed for each controller based on the given time
        {
#ifdef DEBUG
            Serial.println("|------------------------------------------------------|");
            Serial.println("| Serial IK Time cmd received                          |");
#endif

            CtrlCmdTimeout.lastMillis = millis();
        }
#ifdef DEBUG
        else if (args[0] == "help") {
            Serial.println(
                "Commands: ping, time, led, data, motor_feedback, can_relay_to_vic, can_relay_mode, "
                "motor_duty_cycle, motor_speed, stop, digit_linear_ac, digit_wrist, ctrl, IKA, IKT");
            Serial.println(
                "ping: Command to ping the mcu. Should respond with 'pong'\n\n time: \n\n led:  \n on - "
                "turns the led on \n off - turns the led off \n toggle - reverses the led's current state "
                "\n\n data: Command to print feedback data from the mcu \n sendEnc - sends encoder feedback "
                "data");
        }
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

bool spiInit(AS5047P* encoder, int8_t spi_clk, int8_t spi_miso, int8_t spi_mosi, int8_t spi_cs) {
    SPI.begin(spi_clk, spi_miso, spi_mosi, spi_cs);
    return encoder->checkSPICon();
}

void Stop() {
    for (int i = 0; i < 4; i++) {
        armMotors[i]->stop();
    }
}
