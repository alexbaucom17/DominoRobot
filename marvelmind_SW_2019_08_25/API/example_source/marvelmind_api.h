#ifndef __MARVELMIND_API_H_
#define __MARVELMIND_API_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#define MM_USB_DEVICE_ADDRESS 255

#define MM_MAX_DEVICES_COUNT 255

bool mmAPIVersion(uint32_t *version);

bool mmOpenPort();
bool mmOpenPortByName(char *portName);
void mmClosePort();

typedef struct {
     uint8_t fwVerMajor;
     uint8_t fwVerMinor;
     uint8_t fwVerMinor2;
     uint8_t fwVerDeviceType;

     uint8_t fwOptions;

     uint32_t cpuId;
} MarvelmindDeviceVersion;
bool mmGetVersionAndId(uint8_t address, MarvelmindDeviceVersion *mmDevVersion);

typedef struct {
     uint8_t address;
     bool isDuplicatedAddress;
     bool isSleeping;

     uint8_t fwVerMajor;
     uint8_t fwVerMinor;
     uint8_t fwVerMinor2;
     uint8_t fwVerDeviceType;

     uint8_t fwOptions;

     uint8_t flags;
} MarvelmindDeviceInfo;
typedef struct {
    uint8_t numDevices;
    MarvelmindDeviceInfo devices[MM_MAX_DEVICES_COUNT];
} MarvelmindDevicesList;
bool mmGetDevicesList(MarvelmindDevicesList *mmDevices);

bool mmWakeDevice(uint8_t address);
bool mmSendToSleepDevice(uint8_t address);

typedef struct {
      uint32_t worktimeSec;
      int8_t rssi;
      int8_t temperature;
      uint16_t voltageMv;
      uint8_t reserved[16];
} MarvelmindBeaconTelemetry;
bool mmGetBeaconTelemetry(uint8_t address, MarvelmindBeaconTelemetry *bTele);

#define MM_LOCATIONS_PACK_SIZE 6
#define MM_USER_PAYLOAD_BUF_SIZE 128
typedef struct {
      uint8_t address;
      uint8_t headIndex;
      int32_t x_mm;
      int32_t y_mm;
      int32_t z_mm;
      uint8_t statusFlags;
      uint8_t quality;
      uint8_t reserved[2];
} MarvelmindDeviceLocation;
typedef struct {
      MarvelmindDeviceLocation pos[MM_LOCATIONS_PACK_SIZE];

      bool lastDistUpdated;
      uint8_t reserved[5];

      uint8_t userPayloadSize;
      uint8_t userPayloadBuf[MM_USER_PAYLOAD_BUF_SIZE];
} MarvelmindLocationsPack;
bool mmGetLastLocations(MarvelmindLocationsPack *posPack);

#define MM_DISTANCES_PACK_MAX_SIZE 16
typedef struct {
      uint8_t addressRx;
      uint8_t headRx;
      uint8_t addressTx;
      uint8_t headTx;
      uint32_t distance_mm;
      uint8_t reserved;
} MarvelmindDistance;
typedef struct {
      uint8_t numDistances;
      MarvelmindDistance distance[MM_DISTANCES_PACK_MAX_SIZE];
} MarvelmindDistances;
bool mmGetLastDistances(MarvelmindDistances *distPack);

bool mmGetUpdateRateSetting(float *updRateHz);
bool mmSetUpdateRateSetting(float *updRateHz);

bool mmAddSubmap(uint8_t submapId);
bool mmDeleteSubmap(uint8_t submapId);
bool mmFreezeSubmap(uint8_t submapId);
bool mmUnfreezeSubmap(uint8_t submapId);

#define MM_SUBMAP_BEACONS_MAX_NUM 4
#define MM_NEARBY_SUBMAPS_MAX_NUM 8
#define MM_SUBMAP_SERVICE_ZONE_MAX_POINTS 8
typedef struct {
     int16_t x;
     int16_t y;
} ServiceZonePoint;
typedef struct {
     uint8_t startingBeacon;// Starting beacon trilateration
     uint8_t startingSet_1;// Starting set of beacons
     uint8_t startingSet_2;
     uint8_t startingSet_3;
     uint8_t startingSet_4;

     bool enabled3d;// 3D navigation
     bool onlyForZ;// Only for Z coordinate

     bool limitationDistanceIsManual;// Limitation distances
     uint8_t maximumDistanceManual_m;// Maximum distance, m

     int16_t submapShiftX_cm;// Submap X shift, cm
     int16_t submapShiftY_cm;// Submap Y shift, cm
     int16_t submapShiftZ_cm;// Submap Z shift, cm
     uint16_t submapRotation_cdeg;// Submap rotation, centidegrees

     int16_t planeQw;// Plane rotation quaternion (QW,QX,QY,QZ) normalized to 10000
     int16_t planeQx;
     int16_t planeQy;
     int16_t planeQz;

     int16_t serviceZoneThickness_cm;// Service zone thickness, cm

     int16_t hedgesHeightFor2D_cm;// Hedges height in 2D mode

     bool frozen;// true - submap is frozen
     bool locked;// true - submap is locked

     bool beaconsHigher;// true - stationary beacons are higher than mobile
     bool mirrored;// true - submap is mirrored

     uint8_t beacons[MM_SUBMAP_BEACONS_MAX_NUM];// list of beacons in submap (0 - none)
     uint8_t nearbySubmaps[MM_NEARBY_SUBMAPS_MAX_NUM];// list of nearby submaps ID's (255 - none)

     uint8_t serviceZonePointsNum;// Number of service zone polygon points
     ServiceZonePoint serviceZonePolygon[MM_SUBMAP_SERVICE_ZONE_MAX_POINTS];
} MarvelmindSubmapSettings;
bool mmGetSubmapSettings(uint8_t submapId, MarvelmindSubmapSettings *submapSettings);
bool mmSetSubmapSettings(uint8_t submapId, MarvelmindSubmapSettings *submapSettings);

#define MM_SENSOR_RX1 0
#define MM_SENSOR_RX2 1
#define MM_SENSOR_RX3 2
#define MM_SENSOR_RX4 3
#define MM_SENSOR_RX5 4
#define MM_US_SENSORS_NUM 5

#define MM_US_FILTER_19KHZ 0
#define MM_US_FILTER_25KHZ 1
#define MM_US_FILTER_31KHZ 2
#define MM_US_FILTER_37KHZ 3
#define MM_US_FILTER_45KHZ 4
#define MM_US_FILTER_56KHZ 5

typedef struct {
     uint16_t txFrequency_hz;
     uint8_t txPeriodsNumber;

     bool rxAmplifierAGC;
     uint16_t rxAmplificationManual;

     bool sensorsNormal[MM_US_SENSORS_NUM];
     bool sensorsFrozen[MM_US_SENSORS_NUM];

     uint8_t rxDSPFilterIndex;
} MarvelmindUltrasoundSettings;
bool mmGetUltrasoundSettings(uint8_t address, MarvelmindUltrasoundSettings *usSettings);
bool mmSetUltrasoundSettings(uint8_t address, MarvelmindUltrasoundSettings *usSettings);

bool mmEraseMap();
bool mmSetDefaultSettings(uint8_t address);

bool mmBeaconsToAxes(uint8_t address_0, uint8_t address_x, uint8_t address_y);

bool mmDeviceIsModem(uint8_t deviceType);
bool mmDeviceIsBeacon(uint8_t deviceType);
bool mmDeviceIsHedgehog(uint8_t deviceType);

void marvelmindAPILoad();
void marvelmindAPIFree();

#endif // __MARVELMIND_API_H_
