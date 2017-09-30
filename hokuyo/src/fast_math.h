#ifndef FAST_MATH_H
#define FAST_MATH_H

#include "global.h"

typedef struct Pt {
	int x, y;
} Pt_t;


struct fastmathTrigo {
	int n;
	double *cos, *sin; 
};


int dist_squared(Pt_t p1, Pt_t p2);
int dist_to_edge(Pt_t p, int largeurX, int largeurY);


struct fastmathTrigo initFastmath(int n, double *angles, double headingError);
void freeFastmath(struct fastmathTrigo s);

double angle(Pt_t p1, Pt_t p2);
double fastCos(struct fastmathTrigo f, int index);
double fastSin(struct fastmathTrigo f, int index);
double modTwoPi(double a);

#endif
