#include "communication.h"
#include "fast_math.h"
#include "global.h"

#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <stdio.h>

extern FILE* logfile;

void sayHello(){
    printf("[HI:)] Hello !\n");
    fflush(stdin);
}

void pushResults(Cluster_t *coords, int nbr, long timestamp) {
    // timestamp not used !! XXX

    char message[50];

    strcpy(message, "[DATA]");

    for(int i=0; i<nbr; i++){
        sprintf(message, "%s%i,%i", message, coords[i].center.x, coords[i].center.y);
        if (i < nbr -1)
            sprintf(message, "%s#", message);
    }

    printf("%s\n", message);
    fflush(stdin);
}

void pushData(Hok_t hok, long* data) {
    // timestamp not used !! XXX
    int j;
    for(j = hok.imin ; j<= hok.imax ; j++){
        data[j] =  data[j]/(long)10;
    }
    char type[10];
    char message[MAX_DATA*50];
    if (hok.c == 'e')
        strcpy(type, "one");
    else
        strcpy(type, "two");
    int i = hok.imin;
    int mid = floor((float)(hok.imax-hok.imin)/3);
    int fin;
    //while(i<= hok.imax){
    for (j=1 ; j<4 ; j++){
        strcpy(message, "[DATA]");
        fin = hok.imin + j * mid;
        sprintf(message, "%s%s#%d#", message, type, j-1);
        if(j == 1)
            sprintf(message, "%s[", message);

        while(i<= fin){

            sprintf(message, "%s[%lf,%ld]", message, hok.angles[i], data[i]);
            sprintf(message, "%s,", message);
            i = i +1;
        }
        if(j == 3)
            sprintf(message, "%s[0,0]]", message);
        printf( "%s\n", message);
        fflush(stdin);
    }

    /*strcpy(message, "[DATA]");
    sprintf(message, "%s%s#[", message, type);
    while(i < hok.imax){
        sprintf(message, "%s[%lf,%ld]", message, hok.angles[i], data[i]);
        sprintf(message, "%s,", message);
        i = i +1;
    }
    printf( "%s[0,0]]\n", message);
    //printf("ok");
    fflush(stdin);*/
}

void pushInfo(char info){
    printf("[INFO]%c\n", info);
    fflush(stdin);
}
