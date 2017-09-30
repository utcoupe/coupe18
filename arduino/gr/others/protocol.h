//
// Created by tfuhrman on 20/04/17.
//

#ifndef OTHERS_PROTOCOL_H
#define OTHERS_PROTOCOL_H

//#include <String>

#include "parameters.h"
#include <stdint.h>

class String;

//todo ack format ?

// BEGIN_ORDERS - Do not remove this comment
#define START               'S'     //no args, start the program
#define HALT                'h'     //no args, halt the program
#define PARAMETER           'p'     //servo_id(int);position(int);value(int), respond TODO, position = open or close
#define SERVO_OPEN          'o'     //servo_id(int), respond ack when done
#define SERVO_CLOSE         'c'     //servo_id(int), respond ack when done
#define SERVO_INIT          'i'     //servo_id(int), respond ack when done
#define GR_SWEEPER          0       //servo controlling the front motor to swallow balls
#define GR_CANON            1       //servo controlling the left motors to throw balls
#define GR_ROCKET           2       //servo controlling the rocket (funny action)
#define GR_LOADER           3       //servo controlling the access to the canon
// END_ORDERS - Do not remove this comment

enum SERVO_POSITION {
    INIT = 0,
    OPEN = 1,
    CLOSE = 2,
    TIMER = 3,
    NB_POS = 4
};

#define MAX_UINT8_T_VALUE   (uint8_t)255

#define ORDER_INDEX (uint8_t)0
#define ID_INDEX    (uint8_t)2

#define MAX_SERVO   (uint8_t)4

extern unsigned char flagArduinoConnected;

void parseAndExecuteOrder(const String& order);

uint8_t getLog10(const uint16_t number);

#endif //OTHERS_PROTOCOL_H
