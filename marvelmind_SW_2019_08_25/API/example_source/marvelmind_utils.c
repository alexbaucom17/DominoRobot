#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "marvelmind_utils.h"
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <time.h>
#endif

void printBoolEnabled(char * prefix,bool v) {
    if (v) {
        printf("%s: enabled\r\n", prefix);
    } else {
        printf("%s: disabled\r\n", prefix);
    }
}

int boolAsInt(bool v) {
    if (v)
        return 1;
    else
        return 0;
}

// Cross platform sleep function
void sleep_ms(int ms) {
    #ifdef WIN32
    Sleep(ms);
    #else
    usleep(ms*1000);
    #endif // WIN32
}

// Trim unprintable characters from the string
void trim(char * const a)
{
    char *p = a, *q = a;
    while (isspace(*q))            ++q;
    while (*q)                     *p++ = *q++;
    *p = '\0';
    while (p > a && isspace(*--p)) *p = '\0';
}

// Returns device type by hardware type id
MMDeviceType getMMDeviceType(uint8_t deviceType) {
    if (mmDeviceIsModem(deviceType)) {
        return modem;
    }
    if (mmDeviceIsBeacon(deviceType)) {
        return beacon;
    }
    if (mmDeviceIsHedgehog(deviceType)) {
        return hedgehog;
    }

    return unknown;
}

// Prints version and ID of the device
void printMMDeviceVersionAndId(MarvelmindDeviceVersion *dv) {
    printf("Version: %d.%02d", (int) dv->fwVerMajor, (int) dv->fwVerMinor);
    printf("%01d", (int) dv->fwVerMinor2);
    //if (dv->fwVerMinor2 != 0) {
    //    printf("%c",(char) (dv->fwVerMinor2+'a' - 1));
    //}
    printf(".%d   CPU ID=%06x", (int) dv->fwVerDeviceType, dv->cpuId);
    printf("\r\n");
}

// Prints device type
void printMMDeviceType(MMDeviceType *dt) {
    switch(*dt) {
        case modem: {
            printf("Device is modem \r\n");
            break;
        }
        case beacon: {
            printf("Device is beacon \r\n");
            break;
        }
        case hedgehog: {
            printf("Device is hedgehog \r\n");
            break;
        }
        default: {
            printf("Unknown device type \r\n");
            break;
        }
    }
}

#ifndef WIN32
double getPassedTime(struct timespec *t1, struct timespec*t2) {
	double t1_fs= t1->tv_nsec/1000000000.0;
	double t2_fs= t2->tv_nsec/1000000000.0;

	double dt_sec= t2->tv_sec - t1->tv_sec;

	return (dt_sec) + (t2_fs - t1_fs);
}
#endif
