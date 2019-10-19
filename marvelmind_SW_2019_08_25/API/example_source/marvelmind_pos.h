#ifndef __MARVELMIND_POS_H_
#define __MARVELMIND_POS_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "marvelmind_api.h"

#define MARVELMIND_POS_READ_RATE 20

typedef enum {
    notRead, readSuccess, readFail
} MMPosReadStatus;
MMPosReadStatus marvelmindLocationsReadIfNeeded();

void initMarvelmindPos();

#endif // __MARVELMIND_POS_H_
