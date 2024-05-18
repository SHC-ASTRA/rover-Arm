// Standard Includes
#include <Arduino.h>
#include <iostream>
#include <string>

// Our own resources
#include "AstraCAN.h"

using namespace std;


#define LED_PIN 13 //Builtin LED pin for Teensy 4.1 (pin 25 for pi Pico)


//Setting up for myCan line
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;

// Function prototypes 
unsigned long lastBlink;
int led = 0;


void setup() {

    //built_in teensy LED
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);//usb serial line
    digitalWrite(LED_PIN, HIGH);

    //blink to signify boot
    delay(2000);
    digitalWrite(LED_PIN, LOW);
    
    myCan.begin();
    myCan.setBaudRate(1000000);
    myCan.setMaxMB(16);
    myCan.enableFIFO();
    myCan.enableFIFOInterrupt();
  
}



void loop(){
  //blink the LED at 1Hz
  if(millis() - lastBlink > 500){
    lastBlink = millis();
    led = !led;
    digitalWrite(LED_PIN, led);
  }
  

  sendHeartbeat(myCan, 1);
  delay(5);
  sendHeartbeat(myCan, 2);
  delay(5);
  sendHeartbeat(myCan, 3);
  delay(5);

}





