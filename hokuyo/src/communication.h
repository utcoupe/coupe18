#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "fast_math.h"
#include "utils.h"
#include <unistd.h>
#include <sys/types.h>

// More information about communication in ../orders.json

void sayHello();
void pushResults(Cluster_t *coords, int nbr, long timestamp);
void pushInfo(char info);
void pushData(Hok_t hok, long *data);

#endif
