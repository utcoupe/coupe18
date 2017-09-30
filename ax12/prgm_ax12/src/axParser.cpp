//
// Created by tfuhrman on 21/04/17.
//

#include <iostream>
#include <string>


#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "axParser.h"
using namespace std;

std::string GetLineFromCin() {
	//cout << "Waiting for order" <<endl;
    std::string line;
    std::getline(std::cin, line);
    return line;
}

axParser::axParser():started(false), baudnum(1),deviceIndex(0), future(std::async(std::launch::async, GetLineFromCin)) {
   //file.open("input.txt", ios::in);
	//int CommStatus;
    //cout << "init" << endl;
	string command ;


	//cout << "\n\nSyncWrite example for Linux\n\n";
    for(int i = 0; i<vals::nbDetection; i++){
		deviceIndex = detection();
		if (deviceIndex != -1 ) break;
		cerr << "Failed to find USB2Dynamixel" << endl;
		cerr << "Trying " << vals::nbDetection-i << " times"<<endl;
	}

    if( dxl_initialize(deviceIndex, baudnum) == 0 )
    {
        cerr <<"Failed to open USB2Dynamixel!\n" ;
        cerr <<"Terminating\n" ;
        return;
    }
    else{
        // cerr << "Succeed to open USB2Dynamixel!\n";
        waitStart();
    }
//vector<int> positions = {150, 0, 300};
	ax ax1(PR_MODULE_GRABBER, vals::posGrabber);
	ax ax2(PR_MODULE_DUMMY , vals::posDummy);
	servos[PR_MODULE_GRABBER] = &ax1;
	servos[PR_MODULE_DUMMY] =&ax2;
	initializeServos();
    
    while(1)
    {
        // Write goal position
		while (cin.good() && started == true) {

			//cout << "ok";
			command = checkOrder();
			parseAndExecuteOrder(command);
			usleep(100);
		}
    }

    dxl_terminate();
    //cout <<"Press Enter key to terminate...\n" ;
    //getchar();
}

void axParser::initializeServos(){
    // cout << "initializeServos" << endl;
	for (std::map<int, int>::const_iterator it=vals::paramValuesW.begin(); it!=vals::paramValuesW.end(); ++it){
        // cout << "Initilizing " << it->first << " with " << it->second << endl;
		dxl_write_word(254, it->first, it->second);
	}
}

string axParser::checkOrder(){
	string command;
	if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
		command = future.get();
		future = std::async(std::launch::async, GetLineFromCin);

	}
	return command;
}

void axParser::waitStart(){
	string command ;
	clock_t t, diff;

	while(started == false){
		cout << "ax12"<<endl;
		t = clock();
		 diff = 0;

		while ( (float)diff*10/CLOCKS_PER_SEC < 0.3 && started == false) {
		   command = checkOrder();
		   //cin >> command;
			if (command != "")
				parseAndExecuteOrder(command, true);
			usleep(100);
			diff = clock() - t;
			//cout << (double)diff*10/CLOCKS_PER_SEC << endl;
		}
	}
}

int axParser::detection(){
	int CommStatus;
	int baudnum = 1;
	for(int j = 0; j<9; j++){
		//cout << "Recherche sur ACM" << j << endl;
		if(dxl_initialize(j, baudnum) == 1){
			dxl_ping(ID_DETECTION);
			usleep(10000);
			CommStatus = dxl_get_result();
			if( CommStatus == COMM_RXSUCCESS )
			{
				//cout << "USB2AX detecte sur ACM" << j << endl ;
				return j;
			}

		}
	}
	return -1;

}



void axParser::changeParameter(int id, int parameter, int value ){
	servos[id]->setPositions(parameter, value);
}
 std::regex pattern { "abc" };
 bool axParser::checkMessage(const string& mes){
	 std::regex pattern { "^(?:([A-Za-z];[0-9]+;[0-9]+)|([S|h];[0-9]+)|(p;[0-9]+;[0-9]+;[a-z];[0-9]+));$" };
	 return std::regex_match(mes, pattern);
 }
//order is order;id_order;id_servo;params

bool axParser::checkAxId(int id){
	for (std::map<int, ax*>::iterator it=servos.begin(); it!=servos.end(); ++it){
		if(id ==  it->first) return true;
	}
	return false;
}

void axParser::parseAndExecuteOrder(const std::string& order, bool startOnly) {
	if (checkMessage(order) ==  false) return;
	int receivedOrder, id_order, id_ax,j=0, i = 0;
	char c = '0';
	int s = order.size();
    int orderChar = order[0];
	if(s > 1 && order[1] == ';'){
		i = 2;
		j = i;
		while(c != ';' && j< s){
			c = order[++j];
		};
	    id_order = std::stoi(order.substr(i,j-1));
		i = j+1;
		j = j+1;
		if (orderChar != 'S' && orderChar !='h') {
			do{
				//cout << "ok\n";
				c = order[++j];
			}while(c != ';' && j < s);
		id_ax = std::stoi(order.substr(i,j-1));

		//cout << "i = " << i << ", j = " << j<< ", size = " << s << endl;
	    receivedOrder = j+1;
		//cout << "id : " << id_ax <<endl;
		if(checkAxId(id_ax) == false) {
            cerr << "undeclared id" << endl;
            return;
        }

		}


	}
	if(startOnly){
		if(orderChar == START){
			//cout << "Receive start order" <<endl;

			started = true;
            cout <<  id_order << ";" << endl;
		}
	}
	else{

	    switch (orderChar) {
	        case START:
	        {
	            // Ack that arduino has started
	            started = true;
                cout <<  id_order << ";" << endl;
	            break;
	        }
	        case HALT:
	        {
	            // Ack that arduino has stopped
				// cout << "Receive halt order" <<endl;
				started = false;
				waitStart();
	            break;
	        }
	        case PARAMETER:
	        {
	            //todo try to be able to use uint8_t
				// cout << "Receive parameter value " <<order[receivedOrder] << ", order " << id_order << endl;
				if (int position = positionToIndex(order[receivedOrder]) != -1){
					int value = std::stoi(order.substr(receivedOrder+2, s-2));
					changeParameter(id_ax, position, value);
				}else{
					cerr << "error on order " << id_order <<endl;
				}
	            break;
	        }
	        case AX12_INIT:
	        {
				cerr << "Going to init position, order " << id_order<<endl;
	            servos[id_ax]->goTo(vals::INIT, id_order);
				break;
	        }
	        case AX12_ONE:
	        {
				cerr << "Going to position one, order " << id_order<<endl;
	            servos[id_ax]->goTo(vals::ONE, id_order);
	            break;
	        }
	        case AX12_TWO:
	        {
				cerr << "Going to position two, order " << id_order<<endl;
	            servos[id_ax]->goTo(vals::TWO, id_order);
	            break;
	        }
	        default:
	            break;
	    }
	}
	//cout << "next\n";
}

int axParser::positionToIndex(char c){
	switch (c) {
		case AX12_INIT:
			return vals::INIT; break;
		case AX12_ONE:
			return vals::ONE; break;
		case AX12_TWO:
			return vals::TWO; break;
		default:
			return -1; break;
	}
}

int axParser::getLog10(const uint16_t number) {
    if(number>=10000) return 5;
    if(number>=1000) return 4;
    if(number>=100) return 3;
    if(number>=10) return 2;
    return 1;
}

/*
receive orders as String, so keep this format
store the received strings in a QueueList (test that it's working and not using too much space)
when processing, copy the String buffer in an internal buffer to avoid any conflict
best is to use a generic parser, but the parser execute orders too, how to split them ??

goal is to integrate OS48 and to compile a chuck of code with a main (not generic at first) and
generic functions to read and to write through the serial port + trigger on S order with response
 */
