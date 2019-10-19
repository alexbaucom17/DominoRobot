#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#ifndef WIN32
#include <pthread.h>
#endif

#define DATA_INPUT_SEMAPHORE "/udp_data_input_semaphore"

typedef union {uint8_t b[2]; uint16_t w;int16_t wi;} uni_8x2_16;
typedef union {uint8_t b[4];float f;uint32_t dw;int32_t dwi;} uni_8x4_32;

struct PositionValue
{
    uint8_t address;
    uint32_t timestamp;
    int32_t x, y, z;

    bool ready;
};

struct MarvelmindUDP
{
// Server URL address
// default: "127.0.0.1"
    const char *serverAddress;

// Server UDP port
// default: 49200
    uint16_t serverPort;

// Address of hedgehog to request (0 = read streaming data for all mobile beacons)
    uint8_t beaconRequestAddress;

//  Rate of requests send
    uint8_t requestRateHz;

// maximum count of measurements of coordinates stored in buffer
// default: 3
    uint8_t maxBufferedPositions;

// buffer of measurements
    struct PositionValue * positionBuffer;

// verbose flag which activate console output
//		default: False
    bool verbose;

//	pause flag. If True, class would not read data
    bool pause;

//  If True, thread would exit from main loop and stop
    bool terminationRequired;

//  receiveDataCallback is callback function to recieve data
    void (*receiveDataCallback)(struct PositionValue position);
    void (*anyInputPacketCallback)();

// private variables
    uint8_t lastValuesCount_;
    bool haveNewValues_;
#ifdef WIN32
    HANDLE thread_;
    CRITICAL_SECTION lock_;
#else
    pthread_t thread_;
    pthread_mutex_t lock_;
#endif
};

struct MarvelmindUDP * createMarvelmindUDP ();
void destroyMarvelmindUDP (struct MarvelmindUDP * udp);
void startMarvelmindUDP (struct MarvelmindUDP * udp);
void printPositionFromMarvelmindUDP (struct MarvelmindUDP * udp,
                                       bool onlyNew);
bool getPositionFromMarvelmindUDP (struct MarvelmindUDP * udp,
                                   uint8_t address,
                                   struct PositionValue * position);
void stopMarvelmindUDP (struct MarvelmindUDP * udp);

#define DEFAULT_UDP_SERVER_ADDRESS "127.0.0.1"
#define DEFAULT_UDP_SERVER_PORT 49100

