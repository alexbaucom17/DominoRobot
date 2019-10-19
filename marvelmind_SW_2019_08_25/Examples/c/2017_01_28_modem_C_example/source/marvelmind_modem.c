#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#ifdef WIN32
#include <windows.h>
#include <process.h>
#else
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#endif // WIN32
#include "marvelmind_modem.h"

//////////////////////////////////////////////////////////////////////////////
// Calculate CRC (Modbus) for array of bytes
// buf: input buffer
// len: size of buffer
// returncode: CRC value
//////////////////////////////////////////////////////////////////////////////
uint16_t CalcCrcModbus_(uint8_t * buf, int len)
{
    uint16_t crc = 0xFFFF;
    int pos;
    for (pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc
        int i;
        for (i = 8; i != 0; i--) // Loop over each bit
        {
            if ((crc & 0x0001) != 0) // If the LSB is set
            {
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else  // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }
    return crc;
}

#ifdef WIN32
#define SERIAL_PORT_HANDLE HANDLE
#define PORT_NOT_OPENED INVALID_HANDLE_VALUE
//////////////////////////////////////////////////////////////////////////////
// Open Serial Port (Windows only)
// portFileName:  alias of port (e.g. "COM3"). Add prefix "\\\\.\\" before alias
//             to open higher ports (e.g. COM12)
// baudrate:   baudRate rate (e.g. 19200)
// verbose:    print errors
// returncode: valid handle if port is successfully opened or
//             INVALID_HANDLE_VALUE on error
//////////////////////////////////////////////////////////////////////////////
HANDLE OpenSerialPort_ (const char * portFileName, uint32_t baudrate,
                        bool verbose)
{
    HANDLE ttyHandle = CreateFile( TEXT(portFileName), GENERIC_READ|GENERIC_WRITE, 0,
                                   NULL, OPEN_EXISTING, 0, NULL);
    if (ttyHandle==INVALID_HANDLE_VALUE)
    {
        if (verbose)
            printf ("Error: unable to open serial connection to port %s "
                  "(possibly serial port is not available)", portFileName);
        return INVALID_HANDLE_VALUE;
    }
    COMMTIMEOUTS timeouts= {200,200,3000,3000,3000};
    bool returnCode=SetCommTimeouts (ttyHandle, &timeouts);
    if (!returnCode)
    {
        if (verbose) puts ("Error: unable to set serial port timeouts");
        CloseHandle (ttyHandle);
        return INVALID_HANDLE_VALUE;
    }
    DCB dcb= {0};
    returnCode=GetCommState (ttyHandle, &dcb);
    if (!returnCode)
    {
        if (verbose) puts ("Error: unable to get serial port parameters");
        CloseHandle (ttyHandle);
        return INVALID_HANDLE_VALUE;
    }
    dcb.BaudRate = baudrate;
    dcb.fAbortOnError=true;
    returnCode=SetCommState (ttyHandle, &dcb);
    if (!returnCode)
    {
        if (verbose) puts ("Error: unable to set serial port parameters");
        CloseHandle (ttyHandle);
        return INVALID_HANDLE_VALUE;
    }
    return ttyHandle;
}
#else
//////////////////////////////////////////////////////////////////////////////
// Converts baudrate value to baudRate code (Linux only)
// baudrate:   value of baudRate rate (e.g. 19200)
// verbose:    show errors
// returncode: code of baudRate rate (e.g. B19200)
//////////////////////////////////////////////////////////////////////////////
uint32_t _GetBaudCode (uint32_t baudrate, bool verbose)
{
    switch (baudrate)
    {
    case 50:
        return B50;
    case 75:
        return B75;
    case 110:
        return B110;
    case 134:
        return B134;
    case 150:
        return B150;
    case 200:
        return B200;
    case 300:
        return B300;
    case 600:
        return B600;
    case 1200:
        return B1200;
    case 1800:
        return B1800;
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    default:
        if (verbose)
            printf ("Warning: unsupported baudrate %u. Using 9600.\n",
                baudrate);
        return B9600;
    }
}

#define SERIAL_PORT_HANDLE int
#define PORT_NOT_OPENED -1
//////////////////////////////////////////////////////////////////////////////
// Open Serial Port (Linux only)
// portFileName:  alias of port (e.g. "/dev/ttyACM0")
// baudrate:   baudRate rate (e.g. 19200)
// verbose:    show errors
// returncode: valid handle if port is successfully opened or -1 on error
//////////////////////////////////////////////////////////////////////////////
int OpenSerialPort_ (const char * portFileName, uint32_t baudrate, bool verbose)
{
    int ttyHandle = open(portFileName, O_RDWR );
    if (ttyHandle<0)
    {
        if (verbose)
            printf ("Error: unable to open serial connection to port %s "
                  "(possibly serial port is not available)", portFileName);
        return -1;
    }
    struct termios ttyCtrl;
    memset (&ttyCtrl, 0, sizeof ttyCtrl);
    if ( tcgetattr ( ttyHandle, &ttyCtrl ) != 0 )
    {
        if (verbose) puts ("Error: unable to get serial port parameters");
        return -1;
    }

    uint32_t baudCode=_GetBaudCode(baudrate, verbose);
    cfsetospeed (&ttyCtrl, baudCode);
    cfsetispeed (&ttyCtrl, baudCode);
    // 8N1, no flow control
    ttyCtrl.c_cflag     &=  ~(PARENB|CSTOPB|CSIZE|CRTSCTS);
    ttyCtrl.c_cflag     |=  CS8;
    // no signaling chars, no echo, no canonical processing
    ttyCtrl.c_lflag     =   0;
    ttyCtrl.c_oflag     =   0; // no remapping, no delays
    ttyCtrl.c_cc[VMIN]      =   255;
    ttyCtrl.c_cc[VTIME]     =   30; // 3 seconds read timeout
    ttyCtrl.c_cflag     |=  CREAD | CLOCAL; // turn on READ & ignore ctrl lines
    ttyCtrl.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
    ttyCtrl.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    ttyCtrl.c_oflag     &=  ~OPOST; // make raw
    tcflush(ttyHandle, TCIFLUSH ); // Flush port
    if (tcsetattr (ttyHandle, TCSANOW, &ttyCtrl) != 0)
    {
        if (verbose) puts ("Error: unable to set serial port parameters");
        return -1;
    }
    return ttyHandle;
}
#endif

//////////////////////////////////////////////////////////////////////////////

static void clearRequest(struct RequestContext *rq)
{
    rq->address= 0;
    rq->state= idle;
    rq->resultCode= 0;
    rq->requestFinishCallback= NULL;
}

static void planRequest(struct RequestContext *rq, uint8_t address, void (*requestFinishCallback)(void *data))
{
    rq->address= address;
    rq->state= inProgress;
    rq->requestFinishCallback= requestFinishCallback;
}

static bool requestNeeded(struct RequestContext *rq)
{
    return (rq->state == inProgress);
}

static void requestFinish(struct RequestContext *rq, uint8_t resultCode, void *data)
{
    rq->state= finished;
    rq->resultCode= resultCode;
    if (rq->requestFinishCallback != NULL)
    {
        rq->requestFinishCallback(data);
    }
}

//////////////////////////////////////////////////////////////////////////////

// Prepare reading request for sending. Returns size of packet
static uint8_t prepareReadRequest(uint8_t address, uint16_t dataCode, uint16_t accessMode, uint8_t *buffer)
{uint16_t crc;

    buffer[0]= address;
    buffer[1]= MM_PACKET_READ;
    buffer[2]= (dataCode & 0xff);
    buffer[3]= ((dataCode>>8) & 0xff);
    buffer[4]= (accessMode & 0xff);
    buffer[5]= ((accessMode>>8) & 0xff);

    crc= CalcCrcModbus_(buffer,6);
    buffer[6]= (crc & 0xff);
    buffer[7]= ((crc>>8) & 0xff);

    return 8;
}

// Prepare writing request for sending. Returns size of packet
static uint8_t prepareWriteRequest(uint8_t address, uint16_t dataCode, uint16_t accessMode, uint8_t *data, uint8_t dataSize, uint8_t *buffer)
{uint8_t i;
 uint16_t crc;

    buffer[0]= address;
    buffer[1]= MM_PACKET_WRITE;
    buffer[2]= (dataCode & 0xff);
    buffer[3]= ((dataCode>>8) & 0xff);
    buffer[4]= (accessMode & 0xff);
    buffer[5]= ((accessMode>>8) & 0xff);

    buffer[6]= dataSize;

    for(i=0;i<dataSize;i++)
     {
       buffer[7+i]= data[i];
     }

    crc= CalcCrcModbus_(buffer,7+dataSize);
    buffer[7+dataSize]= (crc & 0xff);
    buffer[8+dataSize]= ((crc>>8) & 0xff);

    return (9+dataSize);
}

// Send buffer with request
static bool sendRequestPacket(SERIAL_PORT_HANDLE ttyHandle, uint8_t *buffer, uint8_t packetSize)
{bool writeSuccessed=true;

#ifdef WIN32
    PurgeComm(ttyHandle,PURGE_RXCLEAR);

    DWORD nBytesToWrite= packetSize;
    DWORD nBytesWritten;
    writeSuccessed=WriteFile (ttyHandle, buffer, nBytesToWrite, &nBytesWritten, NULL);
    if (writeSuccessed)
        writeSuccessed= (nBytesToWrite == nBytesWritten);
#else
    tcflush(ttyHandle,TCIOFLUSH);

    int32_t nBytesWritten;
    nBytesWritten=write(ttyHandle, buffer, packetSize);
    if (nBytesWritten != packetSize) writeSuccessed=false;
#endif
    return writeSuccessed;
}

// Receive answer on request to buffer
static uint8_t receiveAnswerPacket(SERIAL_PORT_HANDLE ttyHandle, uint16_t accessMode, uint8_t *buffer, uint8_t packetSize)
{bool readSuccessed=true;
 uint8_t i;
 uint8_t packetSizeToRead;

    if (accessMode == MM_ACCESS_MODE_REMOTE)
    {// In remote mode two replies should be received: first reply from modem, and second from remote device
        packetSizeToRead= packetSize*2;
    }
    else
    {
         packetSizeToRead= packetSize;
    }

#ifdef WIN32
    DWORD nBytesToRead= packetSizeToRead;
    DWORD nBytesRead;
    readSuccessed=ReadFile (ttyHandle, buffer, nBytesToRead, &nBytesRead, NULL);
#else
    int32_t nBytesRead;
    nBytesRead=read(ttyHandle, buffer, packetSizeToRead);
    if (nBytesRead<0) readSuccessed=false;
#endif
    if (!readSuccessed)
        return 0;

    if (accessMode == MM_ACCESS_MODE_REMOTE)
    {// In remote mode two replies should be received: first reply from modem, and second from remote device
        if (nBytesRead<=packetSize)
        {
            return 0;// no data from remote device
        }
        // skip modem reply
        nBytesRead-= packetSize;
        for(i=0;i<nBytesRead;i++)
        {
            buffer[i]= buffer[i+packetSize];
        }
    }

    return nBytesRead;
}

static uint8_t checkReceivedPacket(uint8_t *buffer, uint8_t packetSize)
{uint16_t crc;

    if (buffer[1]&0x80)
    {// if 7-th bit in byte 1 is set, it should be special 5-byte answer with code of error
        packetSize= 5;
    }

    crc= CalcCrcModbus_(buffer, packetSize);
    if (crc != 0)
    {// CRC failed
        return MM_ERROR_CRC;
    }

    if (buffer[1]&0x80)
    {// return received code of error
        return (buffer[2]|MM_ERROR_ANSWERED_MASK);
    }

    if (buffer[1] == MM_PACKET_MODEM_REPLY)
    {
        return MM_ERROR_UNEXPECTED_MODEM_REPLY;
    }

    return MM_EXCHANGE_OK;
}

// receive beacon state data
static uint8_t getBeaconState(SERIAL_PORT_HANDLE ttyHandle, struct BeaconState* beaconState)
{uint8_t buffer[128];
 uint8_t sendPacketSize,receivedPacketSize;
 uint16_t accessMode= MM_ACCESS_MODE_BUFFER;
 uint8_t resCode;
 uint8_t ofs;
 int32_t rssi;
 uint16_t vcc;

    // Send request
    sendPacketSize= prepareReadRequest(beaconState->rq.address, MM_DCODE_GET_BEACON_STATE, accessMode, buffer);
    if (!sendRequestPacket(ttyHandle, &buffer[0], sendPacketSize))
    {// failed to send request
        return MM_ERROR_TRANSMIT;
    }

    // Read answer
    receivedPacketSize= receiveAnswerPacket(ttyHandle, accessMode, buffer, 37);
    if (receivedPacketSize == 0)
    {
        return MM_ERROR_RECEIVE;
    }

    resCode= checkReceivedPacket(buffer, receivedPacketSize);
    if (resCode != MM_EXCHANGE_OK)
    {
        return resCode;
    }

    ofs= MM_RECEIVED_DATA_OFFSET;
    beaconState->workingTimeSeconds= buffer[ofs+0] +
                        (((uint32_t) buffer[ofs+1])<<8) +
                        (((uint32_t) buffer[ofs+2])<<16) +
                        (((uint32_t) buffer[ofs+3])<<24);
    rssi= buffer[ofs+4];
    if (rssi>128)
    {
        beaconState->RSSI_dbM= ((rssi - 256)/2) - 74;
    }
    else
    {
        beaconState->RSSI_dbM= (rssi/2) - 74;
    }
    vcc= buffer[ofs+7] + (((uint16_t) buffer[ofs+8])<<8);
    beaconState->VCC_mV= vcc&0x0fff;
    beaconState->lowPowerFlag= ((vcc&0x4000) != 0);
    beaconState->veryLowPowerFlag= ((vcc&0x8000) != 0);

    return MM_EXCHANGE_OK;
}

// Send sleep control command
static uint8_t sendSleepControlCmd(SERIAL_PORT_HANDLE ttyHandle, struct BeaconSleepControl* beaconSleepControl)
{uint8_t buffer[128];
 uint8_t sendDataBuffer[8];
 uint8_t sendPacketSize,receivedPacketSize;
 uint16_t accessMode;
 uint8_t resCode;

    // fixed signature (password)
    sendDataBuffer[0]= 0x2d;
    sendDataBuffer[1]= 0x94;
    sendDataBuffer[2]= 0x5e;
    sendDataBuffer[3]= 0x81;

    sendDataBuffer[4]= beaconSleepControl->sleepControlCmd;
    sendDataBuffer[5]= 0;
    sendDataBuffer[6]= 0;
    sendDataBuffer[7]= 0;

    if (beaconSleepControl->sleepControlCmd == SCC_WAKE)
    {
        accessMode= MM_ACCESS_MODE_BUFFER;
    }
    else
    {
        accessMode= MM_ACCESS_MODE_REMOTE;
    }

    // Send request
    sendPacketSize= prepareWriteRequest(beaconSleepControl->rq.address, MM_DCODE_SLEEP_WAKE, accessMode, sendDataBuffer, 8, buffer);
    if (!sendRequestPacket(ttyHandle, &buffer[0], sendPacketSize))
    {// failed to send request
        return MM_ERROR_TRANSMIT;
    }

    // Read answer
    receivedPacketSize= receiveAnswerPacket(ttyHandle, accessMode, buffer, 8);
    if (receivedPacketSize == 0)
    {
        return MM_ERROR_RECEIVE;
    }

    resCode= checkReceivedPacket(buffer, receivedPacketSize);
    if (resCode != MM_EXCHANGE_OK)
    {
        return resCode;
    }

    return MM_EXCHANGE_OK;
}

//////////////////////////////////////////////////////////////////////////////
// Thread function started by MarvelmindModem_start
//////////////////////////////////////////////////////////////////////////////

void
#ifndef WIN32
*
#endif // WIN32
Marvelmind_Thread_ (void* param)
{
    struct MarvelmindModem * modem=(struct MarvelmindModem*) param;
    uint8_t resultCode;

    SERIAL_PORT_HANDLE ttyHandle=OpenSerialPort_(modem->ttyFileName,
                                 9600, modem->verbose);
    if (ttyHandle==PORT_NOT_OPENED) modem->terminationRequired=true;
    else if (modem->verbose) printf ("Opened serial port %s\n",modem->ttyFileName);

    while (modem->terminationRequired==false)
    {
        if (requestNeeded(&modem->beaconState.rq))
        {
            resultCode= getBeaconState(ttyHandle,&modem->beaconState);
            requestFinish(&modem->beaconState.rq, resultCode, (void *) &modem->beaconState);
        }

        if (requestNeeded(&modem->beaconSleepControl.rq))
        {
            resultCode= sendSleepControlCmd(ttyHandle,&modem->beaconSleepControl);
            requestFinish(&modem->beaconSleepControl.rq, resultCode, (void *) &modem->beaconSleepControl);
        }
    }
#ifndef WIN32
    return NULL;
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Create an initialize MarvelmindModem structure
// return code: pointer to structure on success or NULL on error
//////////////////////////////////////////////////////////////////////////////

struct MarvelmindModem * createMarvelmindModem ()
{
    struct MarvelmindModem * modem=malloc (sizeof (struct MarvelmindModem));
    if (modem)
    {
        modem->ttyFileName=DEFAULT_TTY_FILENAME;
        modem->verbose=false;
        modem->terminationRequired= false;
#ifdef WIN32
        InitializeCriticalSection(&modem->lock_);
#else
        pthread_mutex_init (&modem->lock_, NULL);
#endif

        clearRequest(&modem->beaconState.rq);
        clearRequest(&modem->beaconSleepControl.rq);
    }
    else puts ("Not enough memory");
    return modem;
}

//////////////////////////////////////////////////////////////////////////////
// Initialize and start work thread
//////////////////////////////////////////////////////////////////////////////
void startMarvelmindModem (struct MarvelmindModem * modem)
{
#ifdef WIN32
    _beginthread (Marvelmind_Thread_, 0, modem);
#else
    pthread_create (&modem->thread_, NULL, Marvelmind_Thread_, modem);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Stop work thread
//////////////////////////////////////////////////////////////////////////////
void stopMarvelmindModem (struct MarvelmindModem * modem)
{
    modem->terminationRequired=true;
    if (modem->verbose) puts ("stopping");
#ifdef WIN32
    WaitForSingleObject (modem->thread_, INFINITE);
#else
    pthread_join (modem->thread_, NULL);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Destroy structures to free memory (You must call stopMarvelmindModem
// first)
//////////////////////////////////////////////////////////////////////////////
void destroyMarvelmindModem (struct MarvelmindModem * modem)
{
    free(modem);
}

//////////////////////////////////////////////////////////////////////////////

// Plan request to get beacon state
void startGetBeaconState(struct MarvelmindModem * modem, uint8_t address, void (*requestFinishCallback)(void *data))
{
#ifdef WIN32
    EnterCriticalSection(&modem->lock_);
#else
    pthread_mutex_lock (&modem->lock_);
#endif

    planRequest(&modem->beaconState.rq, address, requestFinishCallback);

#ifdef WIN32
    LeaveCriticalSection(&modem->lock_);
#else
    pthread_mutex_unlock (&modem->lock_);
#endif
}

//////////////////////////////////////////////////////////////////////////////

// Plan request to send sleep control command
void startSendSleepControlCmd(struct MarvelmindModem * modem, uint8_t address, uint8_t sleepControlCmd, void (*requestFinishCallback)(void *data))
{
#ifdef WIN32
    EnterCriticalSection(&modem->lock_);
#else
    pthread_mutex_lock (&modem->lock_);
#endif

    modem->beaconSleepControl.sleepControlCmd= sleepControlCmd;
    planRequest(&modem->beaconSleepControl.rq, address, requestFinishCallback);

#ifdef WIN32
    LeaveCriticalSection(&modem->lock_);
#else
    pthread_mutex_unlock (&modem->lock_);
#endif
}
