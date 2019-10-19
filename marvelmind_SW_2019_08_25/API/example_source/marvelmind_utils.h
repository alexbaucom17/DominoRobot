#ifndef __MARVELMIND_UTILS_H_
#define __MARVELMIND_UTILS_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "marvelmind_api.h"

typedef enum {
    modem, beacon, hedgehog, unknown
} MMDeviceType;

void printBoolEnabled(char * prefix,bool v);

int boolAsInt(bool v);

void sleep_ms(int ms);
void trim(char * const a);

MMDeviceType getMMDeviceType(uint8_t deviceType);

void printMMDeviceVersionAndId(MarvelmindDeviceVersion *dv);
void printMMDeviceType(MMDeviceType *dt);

#ifndef WIN32
double getPassedTime(struct timespec *t1, struct timespec*t2);
#endif

#endif // __MARVELMIND_UTILS_H_
