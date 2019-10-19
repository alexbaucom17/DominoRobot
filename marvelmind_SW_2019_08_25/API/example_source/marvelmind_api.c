#include "marvelmind_api.h"
#ifdef WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#ifdef WIN32
HINSTANCE mmLibrary;
#else
void* mmLibrary;
#endif

typedef bool __attribute__((stdcall)) (*pt_mm_api_version)(void *pdata);
static pt_mm_api_version pmm_api_version= NULL;
bool mmAPIVersion(uint32_t *version) {
  if (pmm_api_version == NULL)
    return false;

  uint8_t buf[8];
  bool res= (*pmm_api_version)(&buf[0]);

  *version= *((uint32_t *) &buf[0]);

  return res;
}

typedef bool __attribute__((stdcall)) (*pt_mm_open_port)(void);
static pt_mm_open_port pmm_open_port= NULL;
bool mmOpenPort() {
  if (pmm_open_port == NULL)
    return false;

  return (*pmm_open_port)();
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_open_port_by_name)(void *pdata);
static pt_mm_open_port_by_name pmm_open_port_by_name= NULL;
bool mmOpenPortByName(char *portName) {
  if (pmm_open_port_by_name == NULL)
    return false;

  uint8_t buf[255];
  uint8_t i;
  for(i=0;i<255;i++) {
    buf[i]= portName[i];
    if (buf[i] == 0)
        break;
  }

  return (*pmm_open_port_by_name)(&buf[0]);
}

//////

typedef void __attribute__((stdcall)) (*pt_mm_close_port)(void);
static pt_mm_close_port pmm_close_port= NULL;
void mmClosePort() {
  if (pmm_close_port == NULL)
    return;

  (*pmm_close_port)();
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_get_version_and_id)(uint8_t address, void *pdata);
static pt_mm_get_version_and_id pmm_get_version_and_id= NULL;
bool mmGetVersionAndId(uint8_t address, MarvelmindDeviceVersion *mmDevVersion) {
  if (pmm_get_version_and_id == NULL)
    return false;

  uint8_t buf[128];
  bool res= (*pmm_get_version_and_id)(address, &buf[0]);

  mmDevVersion->fwVerMajor= buf[0];
  mmDevVersion->fwVerMinor= buf[1];
  mmDevVersion->fwVerMinor2= buf[2];
  mmDevVersion->fwVerDeviceType= buf[3];
  mmDevVersion->fwOptions= buf[4];

  mmDevVersion->cpuId= *((uint32_t *) &buf[5]);

  return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_get_devices_list)(void *pdata);
static pt_mm_get_devices_list pmm_get_devices_list= NULL;
bool mmGetDevicesList(MarvelmindDevicesList *mmDevices) {
  if (pmm_get_devices_list == NULL)
    return false;

  uint8_t buf[(MM_MAX_DEVICES_COUNT+1)*10];

  bool res= (*pmm_get_devices_list)(&buf[0]);

  if (res) {
    uint8_t i;
    MarvelmindDeviceInfo *devPtr;

    mmDevices->numDevices= buf[0];

    uint32_t ofs= 1;
    for(i=0;i<mmDevices->numDevices;i++) {
        devPtr= &mmDevices->devices[i];

        devPtr->address= buf[ofs+0];
        devPtr->isDuplicatedAddress= (bool) buf[ofs+1];
        devPtr->isSleeping= (bool) buf[ofs+2];

        devPtr->fwVerMajor= buf[ofs+3];
        devPtr->fwVerMinor= buf[ofs+4];
        devPtr->fwVerMinor2= buf[ofs+5];
        devPtr->fwVerDeviceType= buf[ofs+6];

        devPtr->fwOptions= buf[ofs+7];

        devPtr->flags= buf[ofs+8];

        ofs+= 9;
    }
  }

  return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_wake_device)(uint8_t address);
static pt_mm_wake_device pmm_wake_device= NULL;
bool mmWakeDevice(uint8_t address) {
    if (pmm_wake_device == NULL)
        return false;

    bool res= (*pmm_wake_device)(address);

    return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_sleep_device)(uint8_t address);
static pt_mm_sleep_device pmm_sleep_device= NULL;
bool mmSendToSleepDevice(uint8_t address) {
    if (pmm_sleep_device == NULL)
        return false;

    bool res= (*pmm_sleep_device)(address);

    return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_get_beacon_tele) (uint8_t address, void *pdata);
static pt_mm_get_beacon_tele pmm_get_beacon_tele= NULL;
bool mmGetBeaconTelemetry(uint8_t address, MarvelmindBeaconTelemetry *bTele) {
    if (pmm_get_beacon_tele == NULL)
        return false;

    uint8_t buf[128];
    uint8_t i;
    bool res= (*pmm_get_beacon_tele)(address, (void *) &buf[0]);

    bTele->worktimeSec= *((uint32_t *) &buf[0]);
    bTele->rssi= *((int8_t *) &buf[4]);
    bTele->temperature= *((int8_t *) &buf[5]);
    bTele->voltageMv= *((uint16_t *) &buf[6]);

    for(i=0;i<16;i++) {
        bTele->reserved[i]= buf[8+i];
    }

    return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_get_last_locations) (void *pdata);
static pt_mm_get_last_locations pmm_get_last_locations= NULL;
bool mmGetLastLocations(MarvelmindLocationsPack *posPack) {
    if (pmm_get_last_locations == NULL)
        return false;

    uint8_t buf[512];
    uint8_t i;
    bool res= (*pmm_get_last_locations)((void *) &buf[0]);

    uint16_t ofs= 0;
    MarvelmindDeviceLocation *ppos;
    for(i=0;i<MM_LOCATIONS_PACK_SIZE;i++) {
        ppos= &posPack->pos[i];

        ppos->address= buf[ofs+0];
        ppos->headIndex= buf[ofs+1];

        ppos->x_mm= *((int32_t *) &buf[ofs+2]);
        ppos->y_mm= *((int32_t *) &buf[ofs+6]);
        ppos->z_mm= *((int32_t *) &buf[ofs+10]);

        ppos->statusFlags= buf[ofs+14];
        ppos->quality= buf[ofs+15];

        ppos->reserved[0]= buf[ofs+16];
        ppos->reserved[1]= buf[ofs+17];

        ofs+= 18;
    }

    posPack->lastDistUpdated= buf[ofs++];
    for(i=0;i<5;i++) {
        posPack->reserved[i]= buf[ofs++];
    }

    posPack->userPayloadSize= buf[ofs++];
    uint8_t n= posPack->userPayloadSize;
    if (n>MM_USER_PAYLOAD_BUF_SIZE) {
        n= MM_USER_PAYLOAD_BUF_SIZE;
    }
    for(i=0;i<n;i++) {
        posPack->userPayloadBuf[i]= buf[ofs++];
    }

    return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_get_last_distances) (void *pdata);
static pt_mm_get_last_distances pmm_get_last_distances= NULL;
bool mmGetLastDistances(MarvelmindDistances *distPack) {
    if (pmm_get_last_distances == NULL)
        return false;

    uint8_t buf[512];
    uint8_t i;
    bool res= (*pmm_get_last_distances)((void *) &buf[0]);

    distPack->numDistances= buf[0];
    if (distPack->numDistances > MM_DISTANCES_PACK_MAX_SIZE) {
        distPack->numDistances= MM_DISTANCES_PACK_MAX_SIZE;
    }

    uint16_t ofs= 1;
    MarvelmindDistance *pdist;
    for(i=0;i<distPack->numDistances;i++) {
        pdist= &distPack->distance[i];

        pdist->addressRx= buf[ofs+0];
        pdist->headRx= buf[ofs+1];
        pdist->addressTx= buf[ofs+2];
        pdist->headTx= buf[ofs+3];

        pdist->distance_mm= *((uint32_t *) &buf[ofs+4]);

        pdist->reserved= buf[ofs+8];

        ofs+= 9;
    }

    return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_get_update_rate_setting) (float *updRateHz);
static pt_mm_get_update_rate_setting pmm_get_update_rate_setting= NULL;
bool mmGetUpdateRateSetting(float *updRateHz) {
    if (pmm_get_update_rate_setting == NULL)
        return false;

    uint8_t buf[8];
    bool res= (*pmm_get_update_rate_setting)((void *) &buf[0]);

    if (res) {
       uint32_t updRate_mHz= *((uint32_t *) &buf[0]);

       *updRateHz= updRate_mHz/1000.0;
    }

    return res;
}

typedef bool __attribute__((stdcall)) (*pt_mm_set_update_rate_setting) (float *updRateHz);
static pt_mm_set_update_rate_setting pmm_set_update_rate_setting= NULL;
bool mmSetUpdateRateSetting(float *updRateHz) {
    if (pmm_set_update_rate_setting == NULL)
        return false;

    uint8_t buf[8];

    uint32_t updRate_mHz= (*updRateHz)*1000.0;
    *((uint32_t *) &buf[0])= updRate_mHz;

    bool res= (*pmm_set_update_rate_setting)((void *) &buf[0]);

    return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_add_submap) (uint8_t submapId);
static pt_mm_add_submap pmm_add_submap= NULL;
bool mmAddSubmap(uint8_t submapId) {
    if (pmm_add_submap == NULL)
        return false;

     bool res= (*pmm_add_submap)(submapId);

     return res;
}

typedef bool __attribute__((stdcall)) (*pt_mm_delete_submap) (uint8_t submapId);
static pt_mm_delete_submap pmm_delete_submap= NULL;
bool mmDeleteSubmap(uint8_t submapId) {
    if (pmm_delete_submap == NULL)
        return false;

     bool res= (*pmm_delete_submap)(submapId);

     return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_freeze_submap) (uint8_t submapId);
static pt_mm_freeze_submap pmm_freeze_submap= NULL;
bool mmFreezeSubmap(uint8_t submapId) {
    if (pmm_freeze_submap == NULL)
        return false;

    bool res= (*pmm_freeze_submap)(submapId);

    return res;
}

typedef bool __attribute__((stdcall)) (*pt_mm_unfreeze_submap) (uint8_t submapId);
static pt_mm_unfreeze_submap pmm_unfreeze_submap= NULL;
bool mmUnfreezeSubmap(uint8_t submapId) {
    if (pmm_unfreeze_submap == NULL)
        return false;

    bool res= (*pmm_unfreeze_submap)(submapId);

    return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_get_submap_settings) (uint8_t submapId, void *pdata);
static pt_mm_get_submap_settings pmm_get_submap_settings= NULL;
bool mmGetSubmapSettings(uint8_t submapId, MarvelmindSubmapSettings *submapSettings) {
    if (pmm_get_submap_settings == NULL)
        return false;

    uint8_t buf[512];
    uint8_t i;
    bool res= (*pmm_get_submap_settings)(submapId, (void *) &buf[0]);

    submapSettings->startingBeacon= buf[0];
    submapSettings->startingSet_1= buf[1];
    submapSettings->startingSet_2= buf[2];
    submapSettings->startingSet_3= buf[3];
    submapSettings->startingSet_4= buf[4];

    submapSettings->enabled3d= (bool) buf[5];
    submapSettings->onlyForZ= (bool) buf[6];

    submapSettings->limitationDistanceIsManual= (bool) buf[7];
    submapSettings->maximumDistanceManual_m= buf[8];

    submapSettings->submapShiftX_cm= *((int16_t *) &buf[9]);
    submapSettings->submapShiftY_cm= *((int16_t *) &buf[11]);
    submapSettings->submapShiftZ_cm= *((int16_t *) &buf[13]);
    submapSettings->submapRotation_cdeg= *((uint16_t *) &buf[15]);

    submapSettings->planeQw= *((int16_t *) &buf[17]);
    submapSettings->planeQx= *((int16_t *) &buf[19]);
    submapSettings->planeQy= *((int16_t *) &buf[21]);
    submapSettings->planeQz= *((int16_t *) &buf[23]);

    submapSettings->serviceZoneThickness_cm= *((int16_t *) &buf[25]);

    submapSettings->hedgesHeightFor2D_cm= *((int16_t *) &buf[27]);

    submapSettings->frozen= (bool) buf[29];
    submapSettings->locked= (bool) buf[30];

    submapSettings->beaconsHigher= (bool) buf[31];
    submapSettings->mirrored= (bool) buf[32];

    uint8_t ofs= 33;
    for(i=0;i<MM_SUBMAP_BEACONS_MAX_NUM;i++) {
        submapSettings->beacons[i]= buf[ofs+i];
    }
    ofs+= MM_SUBMAP_BEACONS_MAX_NUM;

    for(i=0;i<MM_NEARBY_SUBMAPS_MAX_NUM;i++) {
        submapSettings->nearbySubmaps[i]= buf[ofs+i];
    }
    ofs+= MM_NEARBY_SUBMAPS_MAX_NUM;

    submapSettings->serviceZonePointsNum= buf[ofs++];
    for(i=0;i<MM_SUBMAP_SERVICE_ZONE_MAX_POINTS;i++) {
        submapSettings->serviceZonePolygon[i].x= *((int16_t *) &buf[ofs]);
        submapSettings->serviceZonePolygon[i].y= *((int16_t *) &buf[ofs+2]);

        ofs+= 4;
    }

    return res;
}


typedef bool __attribute__((stdcall)) (*pt_mm_set_submap_settings) (uint8_t submapId, void *pdata);
static pt_mm_set_submap_settings pmm_set_submap_settings= NULL;
bool mmSetSubmapSettings(uint8_t submapId, MarvelmindSubmapSettings *submapSettings) {
    if (pmm_set_submap_settings == NULL)
        return false;

    uint8_t buf[512];
    uint8_t i;

    buf[0]= submapSettings->startingBeacon;
    buf[1]= submapSettings->startingSet_1;
    buf[2]= submapSettings->startingSet_2;
    buf[3]= submapSettings->startingSet_3;
    buf[4]= submapSettings->startingSet_4;

    buf[5]= (uint8_t) submapSettings->enabled3d;
    buf[6]= (uint8_t) submapSettings->onlyForZ;

    buf[7]= (uint8_t) submapSettings->limitationDistanceIsManual;
    buf[8]= submapSettings->maximumDistanceManual_m;

    *((int16_t *) &buf[9])= submapSettings->submapShiftX_cm;
    *((int16_t *) &buf[11])= submapSettings->submapShiftY_cm;
    *((int16_t *) &buf[13])= submapSettings->submapShiftZ_cm;
    *((uint16_t *) &buf[15])= submapSettings->submapRotation_cdeg;

    *((int16_t *) &buf[17])= submapSettings->planeQw;
    *((int16_t *) &buf[19])= submapSettings->planeQx;
    *((int16_t *) &buf[21])= submapSettings->planeQy;
    *((int16_t *) &buf[23])= submapSettings->planeQz;

    *((int16_t *) &buf[25])= submapSettings->serviceZoneThickness_cm;

    *((int16_t *) &buf[27])= submapSettings->hedgesHeightFor2D_cm;

    buf[29]= (uint8_t) submapSettings->frozen;
    buf[30]= (uint8_t) submapSettings->locked;

    buf[31]= (uint8_t) submapSettings->beaconsHigher;
    buf[32]= (uint8_t) submapSettings->mirrored;

    uint8_t ofs= 33;
    for(i=0;i<MM_SUBMAP_BEACONS_MAX_NUM;i++) {
        buf[ofs+i]= submapSettings->beacons[i];
    }
    ofs+= MM_SUBMAP_BEACONS_MAX_NUM;

    for(i=0;i<MM_NEARBY_SUBMAPS_MAX_NUM;i++) {
        buf[ofs+i]= submapSettings->nearbySubmaps[i];
    }
    ofs+= MM_NEARBY_SUBMAPS_MAX_NUM;

    buf[ofs++]= submapSettings->serviceZonePointsNum;
    for(i=0;i<MM_SUBMAP_SERVICE_ZONE_MAX_POINTS;i++) {
        *((int16_t *) &buf[ofs])= submapSettings->serviceZonePolygon[i].x;
        *((int16_t *) &buf[ofs+2])= submapSettings->serviceZonePolygon[i].y;

        ofs+= 4;
    }

    bool res= (*pmm_set_submap_settings)(submapId, (void *) &buf[0]);

    return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_get_ultrasound_settings) (uint8_t address, void *pdata);
static pt_mm_get_ultrasound_settings pmm_get_ultrasound_settings= NULL;
bool mmGetUltrasoundSettings(uint8_t address, MarvelmindUltrasoundSettings *usSettings) {
    if (pmm_get_ultrasound_settings == NULL)
        return false;

    uint8_t buf[64];
    uint8_t i;
    bool res= (*pmm_get_ultrasound_settings)(address, (void *) &buf[0]);

    if (res) {
        usSettings->txFrequency_hz= *((uint16_t *) &buf[0]);
        usSettings->txPeriodsNumber= buf[2];

        usSettings->rxAmplifierAGC= (bool) buf[3];
        usSettings->rxAmplificationManual= *((uint16_t *) &buf[4]);

        for(i=0;i<MM_US_SENSORS_NUM;i++) {
            usSettings->sensorsNormal[i]= (bool) buf[6+i];
        }
        for(i=0;i<MM_US_SENSORS_NUM;i++) {
            usSettings->sensorsFrozen[i]= (bool) buf[11+i];
        }

        usSettings->rxDSPFilterIndex= buf[16];
    }

    return res;
}

typedef bool __attribute__((stdcall)) (*pt_mm_set_ultrasound_settings) (uint8_t address, void *pdata);
static pt_mm_set_ultrasound_settings pmm_set_ultrasound_settings= NULL;
bool mmSetUltrasoundSettings(uint8_t address, MarvelmindUltrasoundSettings *usSettings) {
    if (pmm_set_ultrasound_settings == NULL)
        return false;

    uint8_t buf[64];
    uint8_t i;

    *((uint16_t *) &buf[0])= usSettings->txFrequency_hz;
    buf[2]= usSettings->txPeriodsNumber;

    buf[3]= (uint8_t) usSettings->rxAmplifierAGC;
    *((uint16_t *) &buf[4])= usSettings->rxAmplificationManual;

    for(i=0;i<MM_US_SENSORS_NUM;i++) {
        buf[6+i]= (uint8_t) usSettings->sensorsNormal[i];
    }
    for(i=0;i<MM_US_SENSORS_NUM;i++) {
        buf[11+i]= (uint8_t) usSettings->sensorsFrozen[i];
    }

    buf[16]= usSettings->rxDSPFilterIndex;

    bool res= (*pmm_set_ultrasound_settings)(address, (void *) &buf[0]);

    return res;
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_erase_map) ();
static pt_mm_erase_map pmm_erase_map= NULL;
bool mmEraseMap() {
    if (pmm_erase_map == NULL)
        return false;

    return (*pmm_erase_map)();
}

typedef bool __attribute__((stdcall)) (*pt_mm_set_default_settings) (uint8_t address);
static pt_mm_set_default_settings pmm_set_default_settings= NULL;
bool mmSetDefaultSettings(uint8_t address) {
    if (pmm_set_default_settings == NULL)
        return false;

    return (*pmm_set_default_settings)(address);
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_beacons_to_axes) (void *pdata);
static pt_mm_beacons_to_axes pmm_beacons_to_axes= NULL;
bool mmBeaconsToAxes(uint8_t address_0, uint8_t address_x, uint8_t address_y) {
    if (pmm_beacons_to_axes == NULL)
        return false;

    uint8_t buf[64];

    buf[0]= address_0;
    buf[1]= address_x;
    buf[2]= address_y;

    return (*pmm_beacons_to_axes)((void *) &buf[0]);
}

//////

typedef bool __attribute__((stdcall)) (*pt_mm_device_is_modem)(uint8_t deviceType);
static pt_mm_device_is_modem pmm_device_is_modem= NULL;
bool mmDeviceIsModem(uint8_t deviceType) {
  if (pmm_device_is_modem == NULL)
    return false;

  return (*pmm_device_is_modem)(deviceType);
}

typedef bool __attribute__((stdcall)) (*pt_mm_device_is_beacon)(uint8_t deviceType);
static pt_mm_device_is_beacon pmm_device_is_beacon= NULL;
bool mmDeviceIsBeacon(uint8_t deviceType) {
  if (pmm_device_is_beacon == NULL)
    return false;

  return (*pmm_device_is_beacon)(deviceType);
}

typedef bool __attribute__((stdcall)) (*pt_mm_device_is_hedgehog) (uint8_t deviceType);
static pt_mm_device_is_hedgehog pmm_device_is_hedgehog= NULL;
bool mmDeviceIsHedgehog(uint8_t deviceType) {
  if (pmm_device_is_hedgehog == NULL)
    return false;

  return (*pmm_device_is_hedgehog)(deviceType);
}

//////

void marvelmindAPILoad() {
#ifdef WIN32
  mmLibrary= LoadLibrary("dashapi.dll");

  pmm_api_version= (pt_mm_api_version ) GetProcAddress(mmLibrary, "mm_api_version");

  pmm_open_port= (pt_mm_open_port ) GetProcAddress(mmLibrary, "mm_open_port");
  pmm_open_port_by_name= (pt_mm_open_port_by_name ) GetProcAddress(mmLibrary, "mm_open_port_by_name");
  pmm_close_port= (pt_mm_close_port ) GetProcAddress(mmLibrary, "mm_close_port");

  pmm_get_version_and_id= (pt_mm_get_version_and_id ) GetProcAddress(mmLibrary, "mm_get_device_version_and_id");
  pmm_get_devices_list= (pt_mm_get_devices_list ) GetProcAddress(mmLibrary, "mm_get_devices_list");

  pmm_wake_device= (pt_mm_wake_device ) GetProcAddress(mmLibrary, "mm_wake_device");
  pmm_sleep_device= (pt_mm_sleep_device ) GetProcAddress(mmLibrary, "mm_send_to_sleep_device");

  pmm_get_beacon_tele= (pt_mm_get_beacon_tele ) GetProcAddress(mmLibrary, "mm_get_beacon_telemetry");

  pmm_get_last_locations= (pt_mm_get_last_locations ) GetProcAddress(mmLibrary, "mm_get_last_locations");
  pmm_get_last_distances= (pt_mm_get_last_distances ) GetProcAddress(mmLibrary, "mm_get_last_distances");

  pmm_get_update_rate_setting= (pt_mm_get_update_rate_setting ) GetProcAddress(mmLibrary, "mm_get_update_rate_setting");
  pmm_set_update_rate_setting= (pt_mm_set_update_rate_setting ) GetProcAddress(mmLibrary, "mm_set_update_rate_setting");

  pmm_add_submap= (pt_mm_add_submap ) GetProcAddress(mmLibrary, "mm_add_submap");
  pmm_delete_submap= (pt_mm_delete_submap ) GetProcAddress(mmLibrary, "mm_delete_submap");

  pmm_freeze_submap= (pt_mm_freeze_submap ) GetProcAddress(mmLibrary, "mm_freeze_submap");
  pmm_unfreeze_submap= (pt_mm_unfreeze_submap ) GetProcAddress(mmLibrary, "mm_unfreeze_submap");

  pmm_get_submap_settings= (pt_mm_get_submap_settings ) GetProcAddress(mmLibrary, "mm_get_submap_settings");
  pmm_set_submap_settings= (pt_mm_set_submap_settings ) GetProcAddress(mmLibrary, "mm_set_submap_settings");

  pmm_get_ultrasound_settings= (pt_mm_get_ultrasound_settings ) GetProcAddress(mmLibrary, "mm_get_ultrasound_settings");
  pmm_set_ultrasound_settings= (pt_mm_set_ultrasound_settings ) GetProcAddress(mmLibrary, "mm_set_ultrasound_settings");

  pmm_erase_map= (pt_mm_erase_map ) GetProcAddress(mmLibrary, "mm_erase_map");
  pmm_set_default_settings= (pt_mm_set_default_settings ) GetProcAddress(mmLibrary, "mm_set_default_settings");

  pmm_beacons_to_axes= (pt_mm_beacons_to_axes ) GetProcAddress(mmLibrary, "mm_beacons_to_axes");

  pmm_device_is_modem= (pt_mm_device_is_modem ) GetProcAddress(mmLibrary, "mm_device_is_modem");
  pmm_device_is_beacon= (pt_mm_device_is_beacon ) GetProcAddress(mmLibrary, "mm_device_is_beacon");
  pmm_device_is_hedgehog= (pt_mm_device_is_hedgehog ) GetProcAddress(mmLibrary, "mm_device_is_hedgehog");
#else
// not WIN32
  mmLibrary = dlopen("libdashapi.so", RTLD_LAZY);

  pmm_api_version= dlsym(mmLibrary, "mm_api_version");

  pmm_open_port= dlsym(mmLibrary, "mm_open_port");
  pmm_close_port= dlsym(mmLibrary, "mm_close_port");

  pmm_get_version_and_id= dlsym(mmLibrary, "mm_get_device_version_and_id");
  pmm_get_devices_list= dlsym(mmLibrary, "mm_get_devices_list");

  pmm_wake_device= dlsym(mmLibrary, "mm_wake_device");
  pmm_sleep_device= dlsym(mmLibrary, "mm_send_to_sleep_device");

  pmm_get_beacon_tele= dlsym(mmLibrary, "mm_get_beacon_telemetry");

  pmm_get_last_locations= dlsym(mmLibrary, "mm_get_last_locations");
  pmm_get_last_distances= dlsym(mmLibrary, "mm_get_last_distances");

  pmm_get_update_rate_setting= dlsym(mmLibrary, "mm_get_update_rate_setting");
  pmm_set_update_rate_setting= dlsym(mmLibrary, "mm_set_update_rate_setting");

  pmm_add_submap= dlsym(mmLibrary, "mm_add_submap");
  pmm_delete_submap= dlsym(mmLibrary, "mm_delete_submap");

  pmm_freeze_submap= dlsym(mmLibrary, "mm_freeze_submap");
  pmm_unfreeze_submap= dlsym(mmLibrary, "mm_unfreeze_submap");

  pmm_get_submap_settings= dlsym(mmLibrary, "mm_get_submap_settings");
  pmm_set_submap_settings= dlsym(mmLibrary, "mm_set_submap_settings");

  pmm_get_ultrasound_settings= dlsym(mmLibrary, "mm_get_ultrasound_settings");
  pmm_set_ultrasound_settings= dlsym(mmLibrary, "mm_set_ultrasound_settings");

  pmm_erase_map= dlsym(mmLibrary, "mm_erase_map");
  pmm_set_default_settings= dlsym(mmLibrary, "mm_set_default_settings");

  pmm_beacons_to_axes= dlsym(mmLibrary, "mm_beacons_to_axes");

  pmm_device_is_modem= dlsym(mmLibrary, "mm_device_is_modem");
  pmm_device_is_beacon= dlsym(mmLibrary, "mm_device_is_beacon");
  pmm_device_is_hedgehog= dlsym(mmLibrary, "mm_device_is_hedgehog");
#endif
}

void marvelmindAPIFree() {
#ifdef WIN32
  FreeLibrary(mmLibrary);
#else
  dlclose(mmLibrary);
#endif
}
