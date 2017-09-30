#ifndef GLOBAL_H
#define GLOBAL_H

#include "fast_math.h"
#include <math.h>
#include <sys/wait.h>

#define DEBUG 1

#define PREFIX "[C-HK]  "
// #define SDL

#define TABLE_X 3000
#define TABLE_Y 2000
#define MAX_DATA 1024
#define MAX_CLUSTERS 50 // nombre maxi de clusters
#define MAX_ROBOTS 4 // au cas où qu'on précise pas à l'appel du programme
#define BORDER_MARGIN 50
#define TIMEOUT 100


#define CONE_DIAM_MAX 690 // à modifier en fonction du cône utilisé
#define CONE_CALIB  CONE_DIAM_MAX/2 // diamètre du cône à l'horizontale de l'hokuyo <=> ø vu par l'hokuyo lorsque l'assiette est bonne
#define CONE_HEIGHT 350// hauteur du cône
#define CONE_X_LEFT 967
#define CONE_X_RIGHT 2033
#define CONE_Y 1420
#define CALIB_X (TABLE_X/2)
#define CALIB_Y (TABLE_Y/2)

#define HOK1_SERIAL "1102605"//one-enemy
#define HOK1_X -40 // old : 25
#define HOK1_Y -40  // old : 25
#define HOK1_A M_PI // orientation
#define HOK1_CONE_MIN (-M_PI/2)
#define HOK1_CONE_MAX (M_PI/2)//(M_PI/2)

#define HOK2_SERIAL "1320252"//two-corner 1320252
#define HOK2_X 3040 // old 3025
#define HOK2_Y 1000
#define HOK2_A M_PI
#define HOK2_CONE_MIN (-M_PI/2)
#define HOK2_CONE_MAX (M_PI/2)

// CLUSTERS :
#define CLUSTER_POINTS_BACKWARDS 15
#define MAX_DIST 100 // entre deux points pour les considérer dans le même cluster, old: 200
#define MAX_SIZE_TO_MERGE 200
#define NB_PTS_MIN 3 // pour qu'un cluster soit gardé

// Position zone escalier
#define STAIRS_Y_MIN 1420
#define STAIRS_X_MIN 967
#define STAIRS_X_MAX 2033

#endif
