//##########################################################
//##                      R O B O T I S                   ##
//##         SyncWrite Example code for Dynamixel.        ##
//##                                           2009.11.10 ##
//##########################################################
#include <time.h>
#include <iostream>
#include <unistd.h>


#include <dynamixel.h>


// Defulat setting
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define CONTROL_PERIOD		(500) // usec (Large value is more slow)
#define P_GOAL_POSITION_L 30

const int ALARM_SHUTDOWN = 0; // 00000101
const int CW_ANGLE_LIMIT  	= 0;
const int CCW_ANGLE_LIMIT =	1023;

void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);
using namespace std;

int main()
{
	int baudnum = 0;
	int deviceIndex = 0; //ACMX
	int CommStatus;
	cout  << "------------AX12 search------------" << endl;
	cout << "ACM number : " ;
	cin >> deviceIndex ;
	/*deviceIndex = detection();
	while (deviceIndex == -1){
		cout << "Failed to find USB2Dynamixel");
		cout <<  "Press Enter key to retry..." << endl;" );
		getchar();
		deviceIndex = detection();
	}
	*/
	///////// Open USB2Dynamixel ////////////
	if( dxl_initialize(deviceIndex, baudnum) == 0 )
	{
		cout <<  "Failed to open USB2Dynamixel!" << endl;
		cout <<  "Press Enter key to terminate..." << endl;
		getchar();
		return 0;
	}
	else
		cout <<  "Succeed to open USB2Dynamixel!\n Looking for AX12... " << endl;

	// Set goal speed
	//dxl_write_word( BROADCAST_ID, P_GOAL_SPEED_L, 0 );
	// Set goal position
	//dxl_write_word( BROADCAST_ID, P_GOAL_POSITION_L, AmpPos );

		/*cout <<  "Press Enter key to continue!(press ESC and Enter to quit)" << endl;" );
		if(getchar() == 0x1b)
			break;
		*/

		for(int i=0; i<256; i++){
			dxl_ping(i);
			//cout << "id : "<< id << endl;
			CommStatus = dxl_get_result();
			if( CommStatus == COMM_RXSUCCESS )
			{
				PrintErrorCode();
				cout << "AX12 found, id = " << i << endl;
			}
			//theta += STEP_THETA;
			usleep(CONTROL_PERIOD);

		}


		/*
		// Entre 0 et 1023
		dxl_write_word(1, 34, 1000);
		dxl_write_word(1, 32, 1000);
		sleep(5);*/
		int id = 5;
		// dxl_write_byte(id, 18, 0);
		// dxl_write_byte(id, 4, 1);
		//dxl_write_word(id, 6, 0); //cw angle limit
		// dxl_write_word(id, 8, 1023);
		// dxl_write_word(id, 34, 1023);
		sleep(3);
		dxl_write_byte(id, 24, 1);
		cout << "ok" << endl;

		/*dxl_write_byte(id, 24, 0);
		dxl_read_byte(id, 2);
		cout << "Version firmware" << dxl_get_rxpacket_parameter(0) << endl;

		while(1)
		{
			// Write goal position
			dxl_write_word(2, P_GOAL_POSITION_L, 1);
			// sleep(1);
			// dxl_read_byte(id, 16);
			cout << "Status level : " << dxl_get_rxpacket_parameter(0) << endl;
			//dxl_read_word(5, 36);
			// cout << "position actuelle = %d " << endl;
			 sleep(3);
			dxl_write_word(2, P_GOAL_POSITION_L, 300);

			// sleep(1);
			// dxl_read_byte(id, 16);
			cout << "Status level : " << dxl_get_rxpacket_parameter(0) << endl;
			sleep(3);
			// dxl_read_word(1, 36);
			// cout << "position actuelle = %d " << endl;",dxl_get_rxpacket_parameter(0));
		}*/
		return 0;


	dxl_terminate();
	cout <<  "Press Enter key to terminate..." << endl;
	getchar();

	return 0;
}


void initialize(int id){
	dxl_write_byte(id, 18, ALARM_SHUTDOWN);
	dxl_write_word(id, 6, CW_ANGLE_LIMIT);
	dxl_write_word(id, 8, CCW_ANGLE_LIMIT);

}

// Print communication result
void PrintCommStatus(int CommStatus)
{
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		cout << "COMM_TXFAIL: Failed transmit instruction packet!" << endl;;
		break;

	case COMM_TXERROR:
		cout << "COMM_TXERROR: Incorrect instruction packet!" << endl;;
		break;

	case COMM_RXFAIL:
		cout << "COMM_RXFAIL: Failed get status packet from device!" << endl;;
		break;

	case COMM_RXWAITING:
		cout << "COMM_RXWAITING: Now recieving status packet!" << endl;;
		break;

	case COMM_RXTIMEOUT:
		cout << "COMM_RXTIMEOUT: There is no status packet!" << endl;;
		break;

	case COMM_RXCORRUPT:
		cout << "COMM_RXCORRUPT: Incorrect status packet!" << endl;;
		break;

	default:
		cout << "This is unknown error code!" << endl;;
		break;
	}
}

// Print error bit of status packet
void PrintErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		cout << "Input voltage error!" << endl;;

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		cout << "Angle limit error!" << endl;;

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		cout << "Overheat error!" << endl;;

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		cout << "Out of range error!" << endl;;

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		cout << "Checksum error!" << endl;;

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		cout << "Overload error!" << endl;;

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		cout << "Instruction code error!" << endl;;
}
