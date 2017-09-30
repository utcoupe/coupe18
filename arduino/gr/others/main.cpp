//
// Created by tfuhrman on 21/04/17.
//
#include <Arduino.h>

#include "sender.h"
#include "parameters.h"
#include "protocol.h"
#include "servo_motors.h"

// Flag to know if a computer is connected to the arduino
static unsigned char flagConnected = 0;

//todo debug level as a parameter

void serialRead() {
    String receivedString;
    receivedString = Serial.readStringUntil('\n');
    receivedString.replace("\n", "");
    if (receivedString != "") {
        parseAndExecuteOrder(receivedString);
    }
}

void setup() {
#ifdef __AVR_ATmega32U4__
    Serial.begin(BAUDRATE);
#else
    Serial.begin(BAUDRATE, SERIAL_TYPE);
#endif
    Serial.setTimeout(50);

    servoAttach();
}

//main loop, first read an order from serial, execute the order and then send all data to send
void loop() {
    static bool stop = false;
    servoTimerUpdate();
    // First step, read an order from serial and execute it
    serialRead();
    if (!flagArduinoConnected) {
        SerialSender::SerialSend(SERIAL_INFO, "%s", ARDUINO_ID);
        delay(1000);
    } else {
        //todo something useful...
//        delay(1000);
    }
    SerialSender::SerialSendTask();
}