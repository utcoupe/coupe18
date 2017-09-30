#include <urg_ctrl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include "hokuyo_config.h"
#include "utils.h"
#include "compat.h"
#include "communication.h"

#ifdef SDL
#include "gui.h"
#endif

// void frame(int nb_robots_to_find);
void frame();
//static int nb_hokuyo = 2;
//static int symetry = 0;
static long timeStart = 0;
static Hok_t hok1, hok2;
FILE* logfile;

void exit_handler() {
	fprintf(logfile, "\n%sClosing lidar(s), please wait...\n", PREFIX);
	if (hok1.urg != 0)
		urg_disconnect(hok1.urg);
	/*
	if (hok2.urg != 0)
		urg_disconnect(hok2.urg); */

	// on ne free rien ? genre nos hok et tout ?

	//fflush(logfile);
	/*if (logfile != NULL){
		fprintf(logfile, "\n%sClosing log file and exiting, please wait...\n", PREFIX);
		fclose(logfile);
	}*/
	// kill(getppid(), SIGUSR1); //Erreur envoyee au pere
}

static void catch_SIGINT(int signal){
	exit(EXIT_FAILURE);
}

int main(int argc, char **argv){
	// Disable buffering on stdout
	setvbuf(stdout, NULL, _IOLBF, 0);

	// int nb_robots_to_find = 4;
	hok1.urg = 0;

	// Open log file
	logfile = fopen("/var/log/utcoupe/hokuyo.log", "a+");
//	logfile = stderr;
	if (logfile == NULL) {
		fprintf(stderr, "Can't open log file (what do you think about beeing a sudoer ? :P )\n");
		exit(EXIT_FAILURE);
	}
	fprintf(logfile, "\n\n===== Starting Hokuyo =====\n");
	sayHello();

	atexit(exit_handler); // en cas de signal de fermeture, on déconnecte proprement

	// if(argc <= 1 || ( strcmp(argv[1], "green") != 0 && strcmp(argv[1], "yellow") ) ){
	// 	// fprintf(stderr, "usage: hokuyo {green|yellow} [nbr_robots]\n");
	// 	fprintf(stderr, "usage: hokuyo {green|yellow}\n");
	// 	exit(EXIT_FAILURE);
	// }

	if (signal(SIGINT, catch_SIGINT) == SIG_ERR) {
		fprintf(stderr, "An error occurred while setting a signal handler for SIGINT.\n");
		exit(EXIT_FAILURE);
	 }

	// if (argc >= 3) {
	// 	nb_robots_to_find = atoi(argv[2]);
	// } else {
	// 	nb_robots_to_find = MAX_ROBOTS;
	// }

	/*char paths[2][SERIAL_STR_LEN];
	strcpy(paths[0], "0");
	strcpy(paths[1], "0")

	if (detectHokuyos(paths, nb_hokuyo)) {
		fprintf(stderr, "Failed to detect hokuyos paths\n");
		exit(EXIT_FAILURE);
	};*/
	//fprintf(logfile, "Hokuyo 1 (corner) detected on port %s\n", paths[0]);
	//fprintf(logfile, "Hokuyo 2 (ennemy side) detected on port %s\n", paths[1]);
	fflush(logfile);

	hok1 = initHokuyo( HOK1_A, HOK1_CONE_MIN, HOK1_CONE_MAX, (Pt_t){HOK1_X, HOK1_Y}, 'c');
	if (detectHokuyos1(&hok1,&hok2)) {
		fprintf(stderr, "Failed to detect hokuyos paths\n");
		//exit(EXIT_FAILURE);
	}
	else
		checkAndConnect(&hok1);
/*****
A remettre si deux hokuyos sur une raspi
*****

	hok2 = initHokuyo( HOK2_A, HOK2_CONE_MIN, HOK2_CONE_MAX, (Pt_t){HOK2_X, HOK2_Y}, 'e');
	if (detectHokuyos1(&hok2, &hok1)) {
		fprintf(stderr, "Failed to detect hokuyos paths\n");
		//exit(EXIT_FAILURE);
	}
	else
		checkAndConnect(&hok2);
*/
	//hok2 = initHokuyo(paths[1], HOK2_A, HOK2_CONE_MIN, HOK2_CONE_MAX, (Pt_t){HOK2_X, HOK2_Y} );
	//checkAndConnect(&hok2);

	#ifdef SDL
	initSDL();
	#endif

	fprintf(logfile, "%sStarting hokuyo :\n" /*"%sLooking for %s robots\n"*/, PREFIX/*, PREFIX, argv[1]*/);
	fflush(stdout);
	timeStart = timeMillis();
	long time_last_try = 0;

	while(1){
		long now = timeMillis();
		if (now - time_last_try > TIMEOUT) {
			fprintf(logfile, "%sChecking hokuyos hok1: %d \n", PREFIX, hok1.isWorking);
			//checkAndConnect(&hok1);
//			checkAndConnect(&hok2);

			// If an Hokuyo isn't connected, we scan the ports and try to reconect
			if (!hok1.isWorking) {
				if (detectHokuyos1(&hok1, &hok2)) {
					fprintf(stderr, "Failed to detect hokuyos paths (looking for Hokuyo 1)\n");
				} else {
					checkAndConnect(&hok1);
					if (!hok1.isWorking) {
						fprintf(logfile, "%sHokuyo 1 not working\n", PREFIX);
					}
				}
			}
/*			if (!hok2.isWorking) {
				if (detectHokuyos1(&hok2, &hok1)) {
					fprintf(stderr, "Failed to detect hokuyos paths (looking for Hokuyo 2)\n");
				} else {
					checkAndConnect(&hok2);
					if (!hok2.isWorking) {
						fprintf(logfile, "%sHokuyo 2 not working\n", PREFIX);
					}
				}
			}
			*/
			/*if (!hok2.isWorking && nb_hokuyo == 2) {
				if (detectHokuyos(paths, 2)) {
					fprintf(stderr, "Failed to detect hokuyos paths (looking for Hokuyo 2)\n");
				} else {
					strcpy(hok1.path, paths[1]);
					checkAndConnect(&hok2);
					if (!hok2.isWorking) {
						// pushInfo('1');
						fprintf(logfile, "%sHokuyo 2 not working\n", PREFIX);
					}
				}
			}*/
			time_last_try = now;
		}
		frame();
		fflush(logfile);

	}
	exit(EXIT_SUCCESS);
}

void frame(){
	int nPts1 = 0  ;
	long data1[MAX_DATA] ;
	if (hok1.isWorking) {
		nPts1 = urg_receiveData(hok1.urg, data1, MAX_DATA);
		if (nPts1 < 1){
			printf("erreur : %s\n", urg_error(hok1.urg));
			hok1.isWorking = 0;
		}
	}
	if (hok1.isWorking )
		pushData(hok1, data1);
/*
	if(hok2.isWorking ){
		nPts2 = urg_receiveData(hok2.urg, data2, MAX_DATA);
		if (nPts2 < 1){
			printf("erreur : %s\n", urg_error(hok2.urg));
			hok2.isWorking = 0;
		}
	}
	if (hok2.isWorking )
		pushData(hok2, data2);
	//pushInfo('9');
	*/
	/*for (i=0; i<MAX_DATA; i++){
		fprintf(stderr, " [%ld] ", data[i]);
	}*/



}
