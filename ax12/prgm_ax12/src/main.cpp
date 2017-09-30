//##########################################################
//##                      R O B O T I S                   ##
//##         SyncWrite Example code for Dynamixel.        ##
//##                                           2009.11.10 ##
//##########################################################
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <termio.h>
#include <time.h>
#include <iostream>
#include "axParser.h"
#include <vector>



#define PI	3.141592f
#define NUM_ACTUATOR		3




// Defulat setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define NUM_ACTUATOR		3 // Number of actuator
#define STEP_THETA			(PI / 100.0f) // Large value is more fast
#define CONTROL_PERIOD		(10000) // usec (Large value is more slow)


void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);
using namespace std;
int main()
{
	axParser ax;
	return 0;
}




// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		cout <<"COMM_TXFAIL: Failed transmit instruction packet!\n";
		break;

	case COMM_TXERROR:
		cout <<"COMM_TXERROR: Incorrect instruction packet!\n";
		break;

	case COMM_RXFAIL:
		cout <<"COMM_RXFAIL: Failed get status packet from device!\n";
		break;

	case COMM_RXWAITING:
		cout <<"COMM_RXWAITING: Now recieving status packet!\n";
		break;

	case COMM_RXTIMEOUT:
		cout <<"COMM_RXTIMEOUT: There is no status packet!\n";
		break;

	case COMM_RXCORRUPT:
		cout <<"COMM_RXCORRUPT: Incorrect status packet!\n";
		break;

	default:
		cout <<"This is unknown error code!\n";
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		cout <<"Input voltage error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		cout <<"Angle limit error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		cout <<"Overheat error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		cout <<"Out of range error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		cout <<"Checksum error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		cout <<"Overload error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		cout <<"Instruction code error!\n";
}
