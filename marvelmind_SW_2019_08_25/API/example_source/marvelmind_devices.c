#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "marvelmind_devices.h"

typedef struct {
    uint8_t numDevices;
    MarvelmindDevice devices[MM_MAX_DEVICES_COUNT];
} MarvelmindDevicesListExt;

static MarvelmindDevicesListExt mmDevList;

#ifdef WIN32
static clock_t prevReadTime;
#else
static struct timespec prevReadTime;
#endif

// Returns true if devices info is same
static bool sameDevice(MarvelmindDeviceInfo *dev1, MarvelmindDeviceInfo *dev2) {
    if (dev1->address != dev2->address) return false;
    if (dev1->isDuplicatedAddress != dev2->isDuplicatedAddress) return false;
    if (dev1->isSleeping != dev2->isSleeping) return false;

    if (dev1->fwVerMajor != dev2->fwVerMajor) return false;
    if (dev1->fwVerMinor != dev2->fwVerMinor) return false;
    if (dev1->fwVerMinor2 != dev2->fwVerMinor2) return false;
    if (dev1->fwVerDeviceType != dev2->fwVerDeviceType) return false;

    if (dev1->fwOptions != dev2->fwOptions) return false;

    if (dev1->flags != dev2->flags) return false;

    return true;
}

// Update device in list
static void updateDevice(uint8_t index, MarvelmindDeviceInfo *info) {
    bool connected= ((info->flags&0x01) != 0);
    mmDevList.devices[index].devConnected= connected;

    printf("Device %d updated\r\n", (int) info->address);

    if (connected) {
        MarvelmindDeviceVersion version;
        if (!mmGetVersionAndId(info->address, &version)) {
            printf("Failed read version of device: %d\r\n", (int) info->address);
            return;
        }
        mmDevList.devices[index].version= version;

        printMMDeviceVersionAndId(&version);
        mmDevList.devices[index].deviceType= getMMDeviceType(version.fwVerDeviceType);
        printMMDeviceType(&mmDevList.devices[index].deviceType);
    } else {
        if (info->isSleeping) {
            printf("Device %d is sleeping\r\n", (int) info->address);
        } else {
            printf("Device %d connecting...\r\n", (int) info->address);
        }
    }

    mmDevList.devices[index].info= *info;
}

// Remove device from list
static void removeDevice(uint8_t index) {
    if (mmDevList.numDevices == 0) return;

    printf("Device updated: %d\r\n", (int) mmDevList.devices[index].info.address);

    uint8_t i;
    mmDevList.numDevices--;
    if (mmDevList.numDevices > 0) {
        for(i=index; i<mmDevList.numDevices;i++) {
            mmDevList.devices[i]= mmDevList.devices[i+1];
        }
    }
}

// Add device to list
static void addDevice(MarvelmindDeviceInfo *info) {
    if (mmDevList.numDevices >= MM_MAX_DEVICES_COUNT)
        return;

    updateDevice(mmDevList.numDevices, info);
    printf("Device added: %d\r\n", (int) info->address);

    MarvelmindDeviceLocation *ppos= &mmDevList.devices[mmDevList.numDevices].pos;
    ppos->x_mm= 0;
    ppos->y_mm= 0;
    ppos->z_mm= 0;

    mmDevList.numDevices++;
}

// Remove devices not present in new list
static void removeAbsentDevices(MarvelmindDevicesList *pNewList) {
     bool cont;
     uint8_t i,j;

     for(i=0;i<mmDevList.numDevices;i++) {
        uint8_t address= mmDevList.devices[i].info.address;
        MarvelmindDeviceInfo *info_i= &mmDevList.devices[i].info;

        cont= false;
        for(j=0;j<pNewList->numDevices;j++) {
            if (sameDevice(info_i,&pNewList->devices[j])) {
                cont= true;
                break;
            }

            if (address == pNewList->devices[j].address) {
                updateDevice(i, &pNewList->devices[j]);
                cont= true;
                break;
            }
        }//for j
        if (cont)
            continue;

        // device not found in new list
        removeDevice(i);
     }//for i
}

// Add new devices from new list
static void addNewDevices(MarvelmindDevicesList *pNewList) {
     bool cont;
     uint8_t i,j;

     for(i=0;i<pNewList->numDevices;i++) {
        uint8_t address= pNewList->devices[i].address;
        MarvelmindDeviceInfo *info_i= &pNewList->devices[i];

        cont= false;
        for(j=0;j<mmDevList.numDevices;j++) {
            if (sameDevice(info_i,&mmDevList.devices[j].info)) {
                cont= true;
                break;
            }

            if (address == mmDevList.devices[j].info.address) {
                updateDevice(j, info_i);
                cont= true;
                break;
            }
        }//for j
        if (cont)
            continue;

        // device not found in current list
        addDevice(info_i);
     }//for i
}

// check lists identity
static bool checkDevicesList(MarvelmindDevicesList *pNewList) {
    uint8_t n= pNewList->numDevices;

    if (n == 0) {
        return true;
    }

    uint8_t i;
    for(i=0;i<n;i++) {
        if (!sameDevice(&pNewList->devices[i], &mmDevList.devices[i].info)) {
            if (pNewList->devices[i].address == mmDevList.devices[i].info.address) {
                updateDevice(i, &pNewList->devices[i]);
            } else {
                return false;
            }
        }
    }//for i

    return true;
}

// Read Marvelmind devices list from modem
void marvelmindDevicesReadIfNeeded() {
	#ifdef WIN32
   clock_t curTime= clock();
   double passedSec= ((double)(curTime - prevReadTime))/CLOCKS_PER_SEC;
   #else
   struct timespec curTime;
   clock_gettime(CLOCK_REALTIME, &curTime);
   double passedSec= getPassedTime(&prevReadTime, &curTime);
   #endif
   if (passedSec<1.0) {
        return;
   }
   
   prevReadTime= curTime;

   MarvelmindDevicesList newList;
   if (!mmGetDevicesList(&newList)) {
        return;// failed read
   }

   if (newList.numDevices == mmDevList.numDevices) {
        // check lists identity
        if (checkDevicesList(&newList))
            return;
   }

   removeAbsentDevices(&newList);
   addNewDevices(&newList);
}

// Get device data structure
MarvelmindDevice *getMarvelmindDevice(uint8_t address) {
    uint8_t i;

    if (mmDevList.numDevices == 0)
        return NULL;

    for(i=0;i<mmDevList.numDevices;i++) {
        if (mmDevList.devices[i].info.address == address) {
            return &mmDevList.devices[i];
        }
    }

    return NULL;
}

// Update device location
MarvelmindDevice *marvelmindUpdateLocation(uint8_t address, MarvelmindDeviceLocation *ppos) {
    MarvelmindDevice *mmDevice;

    mmDevice= getMarvelmindDevice(address);
    if (mmDevice == NULL)
        return NULL;

    if (mmDevice->pos.x_mm == ppos->x_mm)
        if (mmDevice->pos.y_mm == ppos->y_mm)
            if (mmDevice->pos.z_mm == ppos->z_mm) {
                return NULL;
            }

    mmDevice->pos= *ppos;

    return mmDevice;
}

// Update distance to device
MarvelmindDevice *marvelmindUpdateDistance(uint8_t addressRx, uint8_t addressTx, uint32_t d) {
    MarvelmindDevice *mmDeviceRx;

    mmDeviceRx= getMarvelmindDevice(addressRx);
    if (mmDeviceRx == NULL)
        return NULL;

    uint32_t dPrev= mmDeviceRx->distances[addressTx].distance_mm;
    if (d == dPrev) {
        return NULL;
    }

    mmDeviceRx->distances[addressTx].distance_mm= d;

    return mmDeviceRx;
}

// Initialize structure of Marvelmind devices
void initMarvelmindDevicesList() {
    mmDevList.numDevices= 0;

#ifdef WIN32
    prevReadTime= clock();
#else
	clock_gettime(CLOCK_REALTIME, &prevReadTime);
#endif

    uint8_t i,j;
    for(i=0;i<MM_MAX_DEVICES_COUNT;i++) {
       for(j=0;j<MM_MAX_DEVICES_COUNT;j++) {
           mmDevList.devices[i].distances[j].distance_mm= 0;
       }
    }
}
