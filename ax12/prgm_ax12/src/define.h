#ifndef DEF_H
#define DEF_H

#include <vector>
#include <map>
// BEGIN_ORDERS - Do not remove this comment
#define START               'S'     //;order_id(int);, start the program
#define HALT                'h'     //;order_id(int);, halt the program
#define PARAMETER           'p'     //;order_id(int);servo_id(int);position(int);value(int) respond TODO, position = open or close
#define AX12_ONE            'o'     //;order_id(int);servo_id(int) respond ack when done
#define AX12_TWO            't'     //;order_id(int);servo_id(int) respond ack when done
#define AX12_INIT           'i'
#define PR_MODULE_DUMMY       6      //id servo controlling the arm which push the module
#define PR_MODULE_GRABBER      5       //id servo controlling the grabber
//END_ORDERS - Do not remove this comment
#define ID_DETECTION 		5




namespace vals{
    const std::vector<int> posDummy = {512, 442, 579}; //init (centre), one (gauche), two (droite, bras vu de face)
    const std::vector<int> posGrabber = {547, 145, 345}; //init (replie), one (ouvert), two (porte)
    const int timeOut = 5;
    const int NB_POS = 3;
    const int INIT = 0;
    const int ONE = 1;
    const int TWO = 2;
    const std::map <int, int> paramValuesW = {
        {32, 1000},      //Moving speed
        {34, 800}      //Torque limit
    };
    const int nbDetection = 300;
};
#endif
