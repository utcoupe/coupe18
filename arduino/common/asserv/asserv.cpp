/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 13/10/13			*
 ****************************************/
  
#include <Arduino.h>
#include "block.h"
#include "compat.h"
#include "parameters.h"
#include "protocol.h"
#include "control.h"
#include "pins.h"
#include "emergency.h"

#include "sender.h"
#include <Timer.h>

/**
 * Main loop function, check if emergency stop and computes the new command to apply.
 * The asserv is working with an internal fifo to execute orders.
 * The loop is activate through a Timer (see arduino library).
 */
void asservLoop();

// Run the loop for asserv at 100 Hz
Timer asservLoopTimer = Timer(DT, &asservLoop);

//TODO make it proper with others
// Flag to know if a computer is connected to the arduino
static unsigned char flagConnected = 0;

//todo debug level as a parameter

/**
 * Read a \n ending string from serial port.
 * The timeout is 50ms.
 * When a string is received, execute the corresponding order.
 */
void serialRead() {
    String receivedString;
    receivedString = Serial.readStringUntil('\n');
    // SerialSender::SerialSend(SERIAL_INFO, receivedString);
    receivedString.replace("\n", "");
    if (receivedString != "") {
        parseAndExecuteOrder(receivedString);
    }
}

/**
 * Arduino setup function, initialize pins and registers.
 */
void setup() {
#ifdef __AVR_ATmega32U4__
    Serial.begin(BAUDRATE);
#else
    Serial.begin(BAUDRATE, SERIAL_TYPE);
#endif
    Serial.setTimeout(50);

#ifdef __AVR_ATmega2560__
	TCCR3B = (TCCR3B & 0xF8) | 0x01 ;
	TCCR1B = (TCCR1B & 0xF8) | 0x01 ;
#else
#ifdef __AVR_ATmega328P__
	TCCR1B = (TCCR1B & 0xF8) | 0x01 ;
#endif
#endif
	initPins();
	ControlInit();

    asservLoopTimer.Start();
}

/**
 * Arduino loop function, read from serial port and send internal serial port data.
 * If it is the time to execute asserv, execute it.
 */
void loop() {
    serialRead();
    if (!flagArduinoConnected) {
        SerialSender::SerialSend(SERIAL_INFO, "%s", ARDUINO_ID);
        delay(100);
    } else {
        asservLoopTimer.Update();
    }
    SerialSender::SerialSendTask();
}

void asservLoop(){

	//Action asserv
	ComputeEmergency();
	ComputeIsBlocked();
	ControlCompute();

    // That's an ugly way to do it, but not working in another way...
    // lastReachedID is defined in control.h file
    if(lastReachedID != 0) {
        SerialSender::SerialSend(SERIAL_INFO, "%d;", (int)lastReachedID);
        lastReachedID = 0;
    }

    ProtocolAutoSendStatus();
}
