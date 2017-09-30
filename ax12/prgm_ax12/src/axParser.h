//
// Created by tfuhrman on 20/04/17.
//

#ifndef OTHERS_PROTOCOL_H
#define OTHERS_PROTOCOL_H

#include "ax.h"
#include <string>
#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <dynamixel.h>
#include <time.h>
#include <unistd.h>
#include <fstream>

#include <thread>
#include <mutex>
#include <regex>
#include <chrono>
#include <future>
#include <map>
#include <vector>
//class String;
//todo ack format ?


#define MAX_UINT8_T_VALUE   (uint8_t)255

#define ORDER_INDEX 0
#define ID_INDEX    2

uint8_t getLog10(const uint16_t number);
class axParser{

    bool started;
    int baudnum, deviceIndex;
    int detection();
    void parseAndExecuteOrder(const std::string& order, bool startOnly = false);
    int getLog10(const uint16_t number);
    void waitStart();
    std::map <int, ax*> servos;
    void changeParameter(int id, int parameter, int value );
    std::future<std::string> future;
    std::string checkOrder();
    bool checkMessage(const std::string& mes);
    int positionToIndex(char c);
    bool checkAxId(int id);
    void initializeServos();
public:
    axParser();
};



// Control table address
#define P_GOAL_POSITION_H	31
#define P_GOAL_SPEED_L		32
#define P_GOAL_SPEED_H		33

#endif //OTHERS_PROTOCOL_H
