#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Types of packets
#define MM_PACKET_READ 0x03
#define MM_PACKET_WRITE 0x10
#define MM_PACKET_MODEM_REPLY 0x7f

// Codes of data
#define MM_DCODE_GET_BEACON_STATE 0x0003
#define MM_DCODE_SET_ADDRESS 0x0101
#define MM_DCODE_ROBOT_STOP 0x0403
#define MM_DCODE_ROBOT_CTRL 0x1000
#define MM_DCODE_RAW_DISTANCES_LAST 0x4000
#define MM_DCODE_RAW_DISTANCES_ALL 0x4001
#define MM_DCODE_COORDINATES 0x4100
#define MM_DCODE_MODEM_CONFIG 0x5000
#define MM_DCODE_SUBMAP_CONFIG_BASE 0x6000
#define MM_DCODE_SLEEP_WAKE 0xb006

// Access modes
#define MM_ACCESS_MODE_DEFAULT 0
#define MM_ACCESS_MODE_REMOTE 1
#define MM_ACCESS_MODE_BUFFER 2

// Result codes
#define MM_EXCHANGE_OK 0
#define MM_ERROR_TRANSMIT 1
#define MM_ERROR_RECEIVE 2
#define MM_ERROR_CRC 3
#define MM_ERROR_UNEXPECTED_MODEM_REPLY 4
#define MM_ERROR_ANSWERED_MASK 0x80

#define MM_RECEIVED_DATA_OFFSET 3

typedef enum {idle, inProgress, finished} RequestState;

struct RequestContext
{
    uint8_t address;
    RequestState state;
    uint8_t resultCode;
    void (*requestFinishCallback)(void *data);
};

// Beacon state structure
struct BeaconState
{
    struct RequestContext rq;

    uint32_t workingTimeSeconds;
    int8_t RSSI_dbM;
    uint16_t VCC_mV;
    bool lowPowerFlag;
    bool veryLowPowerFlag;
};

// Sleep control commands
#define SCC_SLEEP_NORMAL 0
#define SCC_SLEEP_DEEP 1
#define SCC_WAKE 2
struct BeaconSleepControl
{
    struct RequestContext rq;

    uint8_t sleepControlCmd;
};

struct MarvelmindModem
{
// serial port device name (physical or USB/virtual). It should be provided as
// an argument:
// /dev/ttyACM0 - typical for Linux / Raspberry Pi
// /dev/tty.usbmodem1451 - typical for Mac OS X
    const char * ttyFileName;

// verbose flag which activate console output
//		default: False
    bool verbose;

//  If True, thread would exit from main loop and stop
    bool terminationRequired;

    struct BeaconState beaconState;
    struct BeaconSleepControl beaconSleepControl;
#ifdef WIN32
    HANDLE thread_;
    CRITICAL_SECTION lock_;
#else
    pthread_t thread_;
    pthread_mutex_t lock_;
#endif
};

struct MarvelmindModem * createMarvelmindModem ();
void destroyMarvelmindModem (struct MarvelmindModem * modem);

void startMarvelmindModem (struct MarvelmindModem * modem);
void stopMarvelmindModem (struct MarvelmindModem * modem);

void startGetBeaconState(struct MarvelmindModem * modem, uint8_t address, void (*requestFinishCallback)(void *data));
void startSendSleepControlCmd(struct MarvelmindModem * modem, uint8_t address, uint8_t sleepControlCmd, void (*requestFinishCallback)(void *data));

#ifdef WIN32
#define DEFAULT_TTY_FILENAME "\\\\.\\COM3"
#else
#define DEFAULT_TTY_FILENAME "/dev/ttyACM0"
#endif // WIN32
