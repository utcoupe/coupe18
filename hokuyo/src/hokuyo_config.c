#include "hokuyo_config.h"
#include "utils.h"
#include "fast_math.h"
#include "global.h"
#include "compat.h"
#include <math.h>

#include <urg_ctrl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>

extern FILE* logfile;

char expected_serial[2][20] = {
	HOK1_SERIAL, //one - enemy
	HOK2_SERIAL, //two - corner
};

#define NR_ACM_TRY 4
#define VVCOMMAND "VV\n"

int detectHokuyos1(Hok_t *hok1, Hok_t *hok2) {
	char base_path[] = "/dev/ttyACMx";
	int i = 42;
	int j;
	int found = 0;
	//hok1->path = "/dev/ttyACMx";
	fprintf(stderr, "Searching hokuyo \n");
	for (j=0; j<NR_ACM_TRY; j++) {
		char *answer, *serial_nr;
		char *try_path = base_path;
		int fd, n, size, k;
		int try = 0;
		//int port_already_used = 0;
		try_path[11] = '0' + j;
		if (strstr(hok2->path, try_path)){
			continue;
		}

		fprintf(stderr, "Opening port %d\n", j);

		// open serial
		fd = open(try_path, O_RDWR | O_NOCTTY | O_NDELAY);
		if (fd == -1) {
			fprintf(stderr, "open_port: Unable to open serial port %s\n", try_path);
			continue;
		}
		set_interface_attribs(fd, B115200, 0);
		set_blocking(fd, 1);
		answer = malloc(1000*sizeof(char));
		while (!found && try<3) {
			try++;
			// ask for serial number


			n = write(fd, VVCOMMAND, sizeof(VVCOMMAND));
			if (n < 0) {
				fprintf(stderr, "Failed to write on serial port %s\n", try_path);
				break;
			}


			if (!answer) {
				fprintf(stderr, "Failed to allocate answer buffer\n");
				exit(EXIT_FAILURE);
			}
			size = 0;
			answer[size] = serial_read(fd);

			// read until \n\n
			do {
				answer[++size] = serial_read(fd);
			} while (!(answer[size] == '\n' && answer[size-1] == '\n') && size <998);
			answer[size] = '\0';
#if DEBUG

			fprintf(stderr, "Answer is: %s\n", answer);

#endif
			if (strstr(answer, "SERI:") == NULL)
			{
				//fprintf(stderr, "FATAL: Hokuyo communication problem, no serial number !\n");
				fprintf(logfile, "FATAL: Hokuyo communication problem, no serial number !\n");

				continue;
			}
			else{
				fprintf(stderr, "strstr: %s\n", strstr(answer, "SERI:"));
				serial_nr = strstr(answer, "SERI:") + sizeof("SERI:");
				k = 0;
				while (serial_nr[++k] != ';');
				serial_nr[k] = '\0';
			}

#if DEBUG
			fprintf(stderr, "Found serial %s on port %s\n i = %d\n", serial_nr, try_path, i);
			//fprintf(stderr, "i = %d\n", i);
#endif

			// check if it is the right hokuyo
			if (strcmp(serial_nr, expected_serial[0]) == 0) {
				strcpy(hok1->path, try_path);
				hok1->c = 'e';

				i = 0;
				found = 1;
			}
			else if (strcmp(serial_nr, expected_serial[1]) == 0) {
				strcpy(hok1->path, try_path);
				hok1->c = 'c';
				i = 1;
				found = 1;
			}


		}
 		if (found) {
#if DEBUG
			fprintf(stderr, "Found hokuyo (%s) on port %s, c = %c\n",  expected_serial[i], try_path, hok1->c);
#endif
			break;
		}

		free(answer);
		close(fd);
	}
	 if (!found) {
	 	return -1;
	 }

	return 0;
}

int detectHokuyos(char (*paths)[SERIAL_STR_LEN], int nr) {
	char base_path[] = "/dev/ttyACMx";
	int i;
	for (i=0; i<nr; i++) {
		char *path = paths[i];
		int j;
		int found = 0;
		fprintf(stderr, "Searching hokuyo %d\n", i);
		for (j=0; j<NR_ACM_TRY; j++) {
			char *answer, *serial_nr;
			char *try_path = base_path;
			int fd, n, size, k;
			int try = 0;
			int port_already_used = 0;
			try_path[11] = '0' + j;
			for (k=0; k<i; k++) {
				if (strstr(paths[k], try_path))
					port_already_used = 1;
			}

			if (port_already_used) {
				continue;
			}

			fprintf(stderr, "Opening port %d\n", j);
			fprintf(stderr, "Try path %s\n", try_path);

			// open serial
			fd = open(try_path, O_RDWR | O_NOCTTY | O_NDELAY);
			if (fd == -1) {
				fprintf(stderr, "open_port: Unable to open serial port %s\n", try_path);
				continue;
			}
			set_interface_attribs(fd, B115200, 0);
			set_blocking(fd, 1);

			while (!found && try<5) {

				// ask for serial number
				answer = malloc(1000*sizeof(char));

				n = write(fd, VVCOMMAND, sizeof(VVCOMMAND));
				if (n < 0) {
					fprintf(stderr, "Failed to write on serial port %s\n", try_path);
					break;
				}


				if (!answer) {
					fprintf(stderr, "Failed to allocate answer buffer\n");
					exit(EXIT_FAILURE);
				}
				size = 0;
				answer[size] = serial_read(fd);

				// read until \n\n
				do {
					answer[++size] = serial_read(fd);
				} while (!(answer[size] == '\n' && answer[size-1] == '\n') && size <999);
				answer[size] = '\0';
				//answer = urg_sensor_serial_id (ho);
#if DEBUG
				fprintf(stderr, "Answer is: %s\n", answer);
				fprintf(stderr, "strstr: %s\n", strstr(answer, "SERI:"));
#endif
				if (strstr(answer, "SERI:") == 0)
				{
					//fprintf(stderr, "FATAL: Hokuyo communication problem, no serial number !\n");
					fprintf(logfile, "FATAL: Hokuyo communication problem, no serial number !\n");
					try++;
					continue;
				}
				serial_nr = strstr(answer, "SERI:") + sizeof("SERI:");
				k = 0;
				while (serial_nr[++k] != ';');
				serial_nr[k] = '\0';

#if DEBUG
				fprintf(stderr, "Found serial %s on port %s\n", serial_nr, try_path);
				//fprintf(stderr, "i = %d\n", i);
#endif

				// check if it is the right hokuyo
				if (strcmp(serial_nr, expected_serial[i]) == 0) {
					strcpy(path, try_path);
					found = 1;
				}else{
					try++;
				}

			}
	 		if (found) {
#if DEBUG
				fprintf(stderr, "Found hokuyo %d (%s) on port %s\n", i, expected_serial[i], path);
#endif
				break;
			}

			free(answer);
			close(fd);
		}
		 if (!found) {
		 	return -1;
		 }
	}
	return 0;
}

Hok_t initHokuyo( double ori, double cone_min, double cone_max, Pt_t pt, char c) {
	Hok_t hok;
	hok.urg = malloc(sizeof(urg_t));
	strcpy(hok.path ,"/dev/ttyACMx");
	hok.orientation = ori;
	hok.pt = pt;
	hok.cone_min = cone_min;
	hok.cone_max = cone_max;
	hok.error = 0;
	hok.fm.n = 0;
	hok.fm.cos = 0;
	hok.fm.sin = 0;
	hok.isWorking = 0;
	hok.c = c;
	return hok;
}

void checkAndConnect(Hok_t *hok) {
	//fprintf(stderr, "check\n");
	//logfile = stderr;
	if (!hok->isWorking) {
		fprintf(logfile, "%sHokuyo not connected, trying to connect to %s\n", PREFIX, hok->path);
		int error = urg_connect(hok->urg, hok->path, 115200);
		if (error < 0) {
			fprintf(logfile, "%sCan't connect to hokuyo : %s\n", PREFIX, urg_error(hok->urg));
			hok->isWorking = 0;
		} else {
			hok->imin = urg_rad2index(hok->urg, hok->cone_min);
			hok->imax = urg_rad2index(hok->urg, hok->cone_max);

			urg_setCaptureTimes(hok->urg, UrgInfinityTimes);
			error = urg_requestData(hok->urg, URG_MD, hok->imin, hok->imax);
			if (error < 0) {
				fprintf(logfile, "%sCan't connect to hokuyo\n", PREFIX);
				hok->isWorking = 0;
			} else {
				fprintf(logfile, "%sHokuyo connected\n", PREFIX);
				hok->isWorking = 1;
				fprintf(logfile, "%sRequesting data on indexes %d to %d from %s OK\n", PREFIX, hok->imin, hok->imax, hok->path);

				hok->nb_data = urg_dataMax(hok->urg);
				double *angles = malloc(hok->nb_data * sizeof(double));

				int i;
				for (i=0; i<hok->nb_data; i++) {
					angles[i] = modTwoPi(urg_index2rad(hok->urg, i) );
				}
				//fprintf(stderr, "angle : %lf\n", angles[2]);
				double *angles1 = malloc(MAX_DATA * sizeof(double));
				/*for (i=0; i<MAX_DATA; i++) {
					angles1[i] = urg_index2deg(hok->urg, i);
				}*/
				double j = hok->cone_min*180/M_PI;
				for (i=hok->imin; i<MAX_DATA; i++) {
					angles1[i] = j;
					j = j + 0.3515625 ;
				}
				hok->angles = angles1;


				hok->fm = initFastmath(hok->nb_data, angles, hok->error);
				//free(angles);

				fprintf(logfile, "%sCalculted sin/cos data for %s\n", PREFIX, hok->path);
			}
		}
	}
}

Hok_t applySymetry(Hok_t hok) {
	hok.pt = (Pt_t) {TABLE_X - hok.pt.x, hok.pt.y};
	hok.orientation = M_PI - hok.orientation;
	double temp = hok.cone_min;
	hok.cone_min = -hok.cone_max;
	hok.cone_max = -temp;
	return hok;
}

// Angles_t frameWizard (Hok_t *hok, int hok_pos, int symetry){
// 	Pt_t pts[MAX_DATA], coneCenter;
// 	Angles_t results;
// 	Cluster_t clusters[MAX_CLUSTERS];
// 	Cluster_simple_t cone;
// 	int nPts = 0, nClusters = 0, distToCone;

// 	if (hok->isWorking) {
// 		// Valeurs de coneCenter et de la zone de l'hokuyo
// 		coneCenter.y = CONE_Y;
// 		if (((hok_pos == 4) && !symetry) || ((hok_pos == 3) && symetry)){ // si on est sur la gauche de la table (vu du public)
// 			hok->zone = (ScanZone_t){ BORDER_MARGIN, TABLE_X/2, BORDER_MARGIN, TABLE_Y-BORDER_MARGIN };
// 			coneCenter.x = CONE_X_LEFT;
// 		} else { // si on est sur la droite
// 			hok->zone = (ScanZone_t){ TABLE_X/2, TABLE_X-BORDER_MARGIN, BORDER_MARGIN, TABLE_Y-BORDER_MARGIN };
// 			coneCenter.x = CONE_X_RIGHT;
// 		}

// 		// Valeur de la distance entre l'Hokuyo et le cône
// 		if (hok_pos == 3){ // si on est sur un côté
// 			distToCone = sqrt(dist_squared((Pt_t) {HOK2_X, HOK2_Y}, (Pt_t) {CONE_X_RIGHT, CONE_Y}));
// 		} else { // si on est dans un coin
// 			distToCone = sqrt(dist_squared((Pt_t) {HOK1_X, HOK1_Y}, (Pt_t) {CONE_X_LEFT, CONE_Y}));
// 		}

// 		if (hok->isWorking) {
// 			nPts = getPoints(*hok, pts);
// 			if (nPts == -1) {
// 				hok->isWorking = 0;
// 				results.pitch = -1;
// 				results.heading = -1;
// 				return results;
// 			}
// 		} else {
// 			results.pitch = -1;
// 			results.heading = -1;
// 			return results; // Hokuyo  disconnected
// 		}

// 		nClusters = getClustersFromPts(pts, nPts, clusters);

// 		cone = findCone(nClusters, clusters, coneCenter);

// 		if ((cone.center.x ==-1) && (cone.center.y ==-1))
// 		{
// 			results.pitch = -2;
// 			results.heading = -2;
// 			return results; // cone lost
// 		}

// 		// Calcul des erreurs (normalement ce sont des tan-1 mais on les néglige vu l'angle)
// 		// --- pitch
// 		results.pitch = ((CONE_CALIB * CONE_HEIGHT/2)/((float)cone.size) - CONE_HEIGHT/2)/(distToCone);
// 		// --- heading
// 		results.heading = (sqrt(dist_squared(cone.center, coneCenter)))/(distToCone);

// 		return results;
// 	} else {
// 		results.pitch = -1;
// 		results.heading = -1;
// 		return results; // Hokuyo  disconnected
// 	}
// }


// void initWizard (Hok_t *hok1, Hok_t *hok2, int symetry){
// 	Angles_t errors;

// 	fprintf(logfile, "%sLaunching initWizard...\n", PREFIX);

// 	// Début de l'initialisation du premier Hokuyo
// 	fprintf(logfile, "%sPut the 1st hokuyo on the platform which is in front of the public,\n", PREFIX);
// 	if (symetry == 0) // On est vert
// 		fprintf(logfile, "%son the left-hand side.\n", PREFIX);
// 	else
// 		fprintf(logfile, "%son the right-hand side.\n", PREFIX);
// 	fprintf(logfile, "%sIt must look to the other side.\n", PREFIX);
// 	fprintf(logfile, "%sOnce done, please plug it into the Rasp (press any key to continue)\n", PREFIX);
// 	getchar(); // en attendant un ENTER
// 	// boucle de vérification qu'il est branché
// 	while (!hok1->isWorking) {
// 		checkAndConnect(hok1);
// 		sleep(1);
// 	}
// 	fprintf(logfile, "%sOk, first hokuyo detected !\n", PREFIX);

// 	fprintf(logfile, "%sPut the mark on the nearest corner of the stairs. (press any key to continue)\n", PREFIX);
// 	getchar(); // en attendant un ENTER
// 	// boucle de vérification de l'assiette
// 	do{
// 		if (hok1->isWorking){
// 			errors = frameWizard (hok1, 4, symetry);

// 			fprintf(logfile, "%f %f\n", errors.pitch, errors.heading);

// 			if ((errors.pitch == -1.) && (errors.heading == -1.)){
// 				fprintf(logfile, "%sHokuyo disconnected\n", PREFIX);
// 				continue;
// 			} else {
// 				if ((errors.pitch == -2.) && (errors.heading == -2.))
// 					fprintf(logfile, "%sCan't see the cone\n", PREFIX);
// 			}
// 		} else {
// 			checkAndConnect(hok1);
// 		}
// 		sleep(1);
// 	} while (pow(errors.pitch, 2) > 0.0125); // trigo, en estimant que nos cylindres font 10cm de haut à max 2m de distance (0.025²) 0.0125
// 	// une fois l'erreur suffisement petite
// 	// prendre l'erreur ce cap
// 	hok1->error = errors.heading;
// 	fprintf(logfile, "%sHeading error is %f grad = %f°\n", PREFIX, hok1->error, (hok1->error*180)/M_PI);
// 	fprintf(logfile, "%sPitch error is %f grad = %f°\n", PREFIX, errors.pitch, (errors.pitch*180)/M_PI);
// 	fprintf(logfile, "%sOk, let's say that's good\n", PREFIX);
// 	fprintf(logfile, "%sThis hokuyo has been correctly configured (press any key to continue)\n", PREFIX);
// 	getchar();


// 	// Début de l'initialisation du second Hokuyo
// 	fprintf(logfile, "%sPut the 2nd hokuyo on the platform which on the side of the area,\n", PREFIX);
// 	if (symetry == 0) // On est vert
// 		fprintf(logfile, "%son the right-hand side.\n", PREFIX);
// 	else
// 		fprintf(logfile, "%son the left-hand side.\n", PREFIX);
// 	fprintf(logfile, "%sIt must look to the other side.\n", PREFIX);
// 	fprintf(logfile, "%sOnce done, please plug it into the Rasp (press any key to continue)\n", PREFIX);
// 	getchar(); // en attendant un ENTER
// 	// boucle de vérification qu'il est branché
// 	while (!hok2->isWorking) {
// 		checkAndConnect(hok2);
// 		sleep(1);
// 	}
// 	fprintf(logfile, "%sOk, 2nd hokuyo detected !\n", PREFIX);

// 	fprintf(logfile, "%sPut the mark on the nearest corner of the stairs. (press any key to continue)\n", PREFIX);
// 	getchar(); // en attendant un ENTER
// 	// boucle de vérification de l'assiette
// 	do{
// 		if (hok2->isWorking){
// 			errors = frameWizard (hok2, 3, symetry);

// 			fprintf(logfile, "%f %f\n", errors.pitch, errors.heading);

// 			if ((errors.pitch == -1.) && (errors.heading == -1.)){
// 				fprintf(logfile, "%sHokuyo disconnected\n", PREFIX);
// 				continue;
// 			} else {
// 				if ((errors.pitch == -2.) && (errors.heading == -2.))
// 					fprintf(logfile, "%sCan't see the cone\n", PREFIX);
// 			}
// 		} else {
// 			checkAndConnect(hok2);
// 		}
// 		sleep(1);
// 	} while (pow(errors.pitch, 2) > 0.0125); // trigo, en estimant que nos cylindres font 10cm de haut à max 2m de distance (0.025²) 0.0125
// 	// une fois l'erreur suffisement petite
// 	// prendre l'erreur ce cap
// 	hok2->error = errors.heading;
// 	fprintf(logfile, "%sHeading error is %f grad = %f°\n", PREFIX, hok2->error, (hok2->error*180)/M_PI);
// 	fprintf(logfile, "%sPitch error is %f grad = %f°\n", PREFIX, errors.pitch, (errors.pitch*180)/M_PI);
// 	fprintf(logfile, "%sOk, let's say that's good\n", PREFIX);
// 	fprintf(logfile, "%sThis hokuyo has been correctly configured (press any key to continue)\n", PREFIX);
// 	getchar();

// 	fprintf(logfile, "%sWaiting for the match to start...\n", PREFIX);
// }
