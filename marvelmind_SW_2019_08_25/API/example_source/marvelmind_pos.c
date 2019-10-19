#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "marvelmind_pos.h"
#include "marvelmind_devices.h"

#ifdef WIN32
static clock_t prevReadTime;
#else
static struct timespec prevReadTime;
#endif

////////////////////////////////////////////////////////////////////////

// Read raw distances between Marvelmind devices
void marvelmindReadRawDistances() {
    MarvelmindDistances distPack;

    if (mmGetLastDistances(&distPack)) {
        uint8_t n= distPack.numDistances;
        uint8_t i;
        for(i=0; i<n; i++) {
            uint8_t addressRx= distPack.distance[i].addressRx;
            uint8_t addressTx= distPack.distance[i].addressTx;
            if ( (addressRx == 0) || (addressTx == 0))
                continue;

            uint32_t dist= distPack.distance[i].distance_mm;

            MarvelmindDevice *mmDeviceRx= marvelmindUpdateDistance(addressRx, addressTx, dist);
            if (mmDeviceRx != NULL) {
                printf("Raw distance: %d ==> %d  : %.3f \r\n", (int) addressTx, (int) addressRx, (float) dist/1000.0);
            }
        }//for i
    }
}

// Read new locations of Marvelmind devices
MMPosReadStatus marvelmindLocationsReadIfNeeded() {
   #ifdef WIN32
   clock_t curTime= clock();
   double passedSec= ((double)(curTime - prevReadTime))/CLOCKS_PER_SEC;
   #else
   struct timespec curTime;
   clock_gettime(CLOCK_REALTIME, &curTime);
   double passedSec= getPassedTime(&prevReadTime, &curTime);
   #endif
   
    if (passedSec<(1.0/MARVELMIND_POS_READ_RATE)) {
        return notRead;
    }
    prevReadTime= curTime;

    MarvelmindLocationsPack posPack;
    if (mmGetLastLocations(&posPack)) {
        uint8_t i;
        MarvelmindDeviceLocation pos;
        MarvelmindDevice *mmDevice;

        if (posPack.lastDistUpdated) {
            marvelmindReadRawDistances();
        }

        for(i=0;i<MM_LOCATIONS_PACK_SIZE;i++) {
            pos= posPack.pos[i];
            if (pos.address == 0)
                continue;

            mmDevice= marvelmindUpdateLocation(pos.address,&pos);
            if (mmDevice == NULL)
                continue;

            if (mmDevice->deviceType == hedgehog) {
                printf("Hedge  %d location: X=%.3f, Y=%.3f, Z=%.3f, quality= %d %%\r\n",
                       (int) pos.address,
                       (float) pos.x_mm/1000.0, (float) pos.y_mm/1000.0, (float) pos.z_mm/1000.0,
                       (int) pos.quality);
            }
            else if (mmDevice->deviceType == beacon) {
                printf("Beacon %d location: X=%.3f, Y=%.3f, Z=%.3f \r\n",
                       (int) pos.address,
                       (float) pos.x_mm/1000.0, (float) pos.y_mm/1000.0, (float) pos.z_mm/1000.0);
            }
        }//for i

        return readSuccess;
    }

    return readFail;
}


// Initialize Marvelmind positions module
void initMarvelmindPos() {
#ifdef WIN32
    prevReadTime= clock();
#else
	clock_gettime(CLOCK_REALTIME, &prevReadTime);
#endif
}
