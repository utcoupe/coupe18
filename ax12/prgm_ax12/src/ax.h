#ifndef OTHERS_AX_H
#define OTHERS_AX_H
//#include "axParser.h"
#include "define.h"
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



using namespace std;
class ax{
    //friend class axParser;
    static const int P_GOAL_POSITION_L = 30;
    static mutex lockSerial;
    static mutex lockCout;
    int id;
    std::atomic<bool> killAction;
    std::mutex m;
    std::vector<int> positions;
    std::mutex lockAction;
    std::vector<std::thread> actions;
    int toAxValue(int val){
        return val*3.41;
    }

    static const int NB_POS = 3;
public:
    ax(int id, std::vector<int> pos):id(id), positions(std::vector<int> (vals ::NB_POS)){
        positions = pos;
        killAction = false;

    }
    void executeAction(int finalPos, int idOrder);
    void goTo(int pos, int idOrder);
     void initialize();
    void setPositions(int i, int value){
        positions[i] = value;
    }
};
#endif
