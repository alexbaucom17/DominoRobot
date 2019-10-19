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
#include <sys/poll.h>
#endif // WIN32
#include "marvelmind_nav/marvelmind_hedge.h"

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
    HANDLE ttyHandle = CreateFile( TEXT(portFileName), GENERIC_READ, 0,
                                   NULL, OPEN_EXISTING, 0, NULL);
    if (ttyHandle==INVALID_HANDLE_VALUE)
    {
        if (verbose)
         {
			puts(portFileName);
            puts ("Error: unable to open serial connection "
                  "(possibly serial port is not available)");
		 }
        return INVALID_HANDLE_VALUE;
    }
    COMMTIMEOUTS timeouts= {3000,3000,3000,3000,3000};
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
    int ttyHandle = open(portFileName, O_RDWR| O_NONBLOCK | O_NDELAY );
    if (ttyHandle<0)
    {
        if (verbose)
         {
			puts(portFileName);
            puts ("Error: unable to open serial connection "
                  "(possibly serial port is not available)");
		 }
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
    ttyCtrl.c_cc[VMIN]      =   0; // read doesn't block
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

//////////////////////////////////////////////////////////////////////////

static int16_t get_int16(uint8_t *buffer)
{
	int16_t res= buffer[0] |
                 (((uint16_t ) buffer[1])<<8);
                 
    return res;
}

static uint32_t get_uint32(uint8_t *buffer)
{
	uint32_t res= buffer[0] |
            (((uint32_t ) buffer[1])<<8) |
            (((uint32_t ) buffer[2])<<16) |
            (((uint32_t ) buffer[3])<<24);
                 
    return res;
}

static int32_t get_int32(uint8_t *buffer)
{
	int32_t res= buffer[0] |
            (((uint32_t ) buffer[1])<<8) |
            (((uint32_t ) buffer[2])<<16) |
            (((uint32_t ) buffer[3])<<24);
                 
    return res;
}

//////////////////////////////////////////////////////////////////////////

static uint8_t markPositionReady(struct MarvelmindHedge * hedge)
{uint8_t ind= hedge->lastValues_next;
 uint8_t indCur= ind;

    hedge->positionBuffer[ind].ready=
        true;
    hedge->positionBuffer[ind].processed=
        false;
    ind++;
    if (ind>=hedge->maxBufferedPositions)
        ind=0;
    if (hedge->lastValuesCount_<hedge->maxBufferedPositions)
        hedge->lastValuesCount_++;
    hedge->haveNewValues_=true;

    hedge->lastValues_next= ind;

    return indCur;
}

static struct PositionValue process_position_datagram(struct MarvelmindHedge * hedge, uint8_t *buffer)
{uint8_t ind= hedge->lastValues_next;

    hedge->positionBuffer[ind].address=
        buffer[16];
    hedge->positionBuffer[ind].timestamp=
        buffer[5] |
        (((uint32_t ) buffer[6])<<8) |
        (((uint32_t ) buffer[7])<<16) |
        (((uint32_t ) buffer[8])<<24);

    int16_t vx= buffer[9] |
                (((uint16_t ) buffer[10])<<8);
    hedge->positionBuffer[ind].x= vx*10;// millimeters

    int16_t vy= buffer[11] |
                (((uint16_t ) buffer[12])<<8);
    hedge->positionBuffer[ind].y= vy*10;// millimeters

    int16_t vz= buffer[13] |
                (((uint16_t ) buffer[14])<<8);
    hedge->positionBuffer[ind].z= vz*10;// millimeters
    
    hedge->positionBuffer[ind].flags= buffer[15];
    
    uint16_t vang= buffer[17] |
                   (((uint16_t ) buffer[18])<<8);
    hedge->positionBuffer[ind].angle= ((float) (vang & 0x0fff))/10.0f;

    hedge->positionBuffer[ind].highResolution= false;

    ind= markPositionReady(hedge);

    return hedge->positionBuffer[ind];
}

static struct PositionValue process_position_highres_datagram(struct MarvelmindHedge * hedge, uint8_t *buffer)
{uint8_t ind= hedge->lastValues_next;

    hedge->positionBuffer[ind].address=
        buffer[22];
    hedge->positionBuffer[ind].timestamp=
        buffer[5] |
        (((uint32_t ) buffer[6])<<8) |
        (((uint32_t ) buffer[7])<<16) |
        (((uint32_t ) buffer[8])<<24);

    int32_t vx= buffer[9] |
                (((uint32_t ) buffer[10])<<8) |
                (((uint32_t ) buffer[11])<<16) |
                (((uint32_t ) buffer[12])<<24);
    hedge->positionBuffer[ind].x= vx;

    int32_t vy= buffer[13] |
                (((uint32_t ) buffer[14])<<8) |
                (((uint32_t ) buffer[15])<<16) |
                (((uint32_t ) buffer[16])<<24);
    hedge->positionBuffer[ind].y= vy;

    int32_t vz= buffer[17] |
                (((uint32_t ) buffer[18])<<8) |
                (((uint32_t ) buffer[19])<<16) |
                (((uint32_t ) buffer[20])<<24);
    hedge->positionBuffer[ind].z= vz;
    
    hedge->positionBuffer[ind].flags= buffer[21];
    
    uint16_t vang= buffer[23] |
                   (((uint16_t ) buffer[24])<<8);
    hedge->positionBuffer[ind].angle= ((float) (vang & 0x0fff))/10.0f;

    hedge->positionBuffer[ind].highResolution= true;

    ind= markPositionReady(hedge);

    return hedge->positionBuffer[ind];
}

static struct StationaryBeaconPosition *getOrAllocBeacon(struct MarvelmindHedge * hedge,uint8_t address)
{
    uint8_t i;
    uint8_t n_used= hedge->positionsBeacons.numBeacons;

    if (n_used != 0)
        for(i=0;i<n_used;i++)
        {
            if (hedge->positionsBeacons.beacons[i].address == address)
            {
                return &hedge->positionsBeacons.beacons[i];
            }
        }

    if (n_used >= (MAX_STATIONARY_BEACONS-1))
        return NULL;

    hedge->positionsBeacons.numBeacons= (n_used + 1);
    return &hedge->positionsBeacons.beacons[n_used];
}

static void process_beacons_positions_datagram(struct MarvelmindHedge * hedge, uint8_t *buffer)
{
    uint8_t n= buffer[5];// number of beacons in packet
    uint8_t i,ofs;
    uint8_t address;
    int16_t x,y,z;
    struct StationaryBeaconPosition *b;

    if ((1+n*8)!=buffer[4])
        return;// incorrect size

    for(i=0;i<n;i++)
    {
        ofs= 6+i*8;

        address= buffer[ofs+0];
        x=  buffer[ofs+1] |
            (((uint16_t ) buffer[ofs+2])<<8);
        y=  buffer[ofs+3] |
            (((uint16_t ) buffer[ofs+4])<<8);
        z=  buffer[ofs+5] |
            (((uint16_t ) buffer[ofs+6])<<8);

        b= getOrAllocBeacon(hedge, address);
        if (b != NULL)
        {
            b->address= address;
            b->x= x*10;// millimeters
            b->y= y*10;// millimeters
            b->z= z*10;// millimeters

            b->highResolution= false;
            b->updatedForMsg= true;

            hedge->positionsBeacons.updated= true;
        }
    }
}

static void process_beacons_positions_highres_datagram(struct MarvelmindHedge * hedge, uint8_t *buffer)
{
    uint8_t n= buffer[5];// number of beacons in packet
    uint8_t i,ofs;
    uint8_t address;
    int32_t x,y,z;
    struct StationaryBeaconPosition *b;

    if ((1+n*14)!=buffer[4])
        return;// incorrect size

    for(i=0;i<n;i++)
    {
        ofs= 6+i*14;

        address= buffer[ofs+0];
        x=  buffer[ofs+1] |
            (((uint32_t ) buffer[ofs+2])<<8) |
            (((uint32_t ) buffer[ofs+3])<<16) |
            (((uint32_t ) buffer[ofs+4])<<24);
        y=  buffer[ofs+5] |
            (((uint32_t ) buffer[ofs+6])<<8) |
            (((uint32_t ) buffer[ofs+7])<<16) |
            (((uint32_t ) buffer[ofs+8])<<24);
        z=  buffer[ofs+9] |
            (((uint32_t ) buffer[ofs+10])<<8) |
            (((uint32_t ) buffer[ofs+11])<<16) |
            (((uint32_t ) buffer[ofs+12])<<24);

        b= getOrAllocBeacon(hedge, address);
        if (b != NULL)
        {
            b->address= address;
            b->x= x;
            b->y= y;
            b->z= z;

            b->highResolution= true;
            b->updatedForMsg= true;

            hedge->positionsBeacons.updated= true;
        }
    }
}

static void process_imu_raw_datagram(struct MarvelmindHedge * hedge, uint8_t *buffer)
{uint8_t *dataBuf= &buffer[5];
	
    hedge->rawIMU.acc_x= get_int16(&dataBuf[0]);
    hedge->rawIMU.acc_y= get_int16(&dataBuf[2]);
    hedge->rawIMU.acc_z= get_int16(&dataBuf[4]);
    
    //
    hedge->rawIMU.gyro_x= get_int16(&dataBuf[6]);
    hedge->rawIMU.gyro_y= get_int16(&dataBuf[8]);
    hedge->rawIMU.gyro_z= get_int16(&dataBuf[10]);
    
    //
    hedge->rawIMU.compass_x= get_int16(&dataBuf[12]);
    hedge->rawIMU.compass_y= get_int16(&dataBuf[14]);
    hedge->rawIMU.compass_z= get_int16(&dataBuf[16]);

    hedge->rawIMU.timestamp= get_uint32(&dataBuf[24]);
    
    hedge->rawIMU.updated= true;
}

static void process_imu_fusion_datagram(struct MarvelmindHedge * hedge, uint8_t *buffer)
{uint8_t *dataBuf= &buffer[5];
	
    hedge->fusionIMU.x= get_int32(&dataBuf[0]);
    hedge->fusionIMU.y= get_int16(&dataBuf[4]);
    hedge->fusionIMU.z= get_int16(&dataBuf[8]);
    
    hedge->fusionIMU.qw= get_int16(&dataBuf[12]);
    hedge->fusionIMU.qx= get_int16(&dataBuf[14]);
    hedge->fusionIMU.qy= get_int16(&dataBuf[16]);
    hedge->fusionIMU.qz= get_int16(&dataBuf[18]);
    
    hedge->fusionIMU.vx= get_int16(&dataBuf[20]);
    hedge->fusionIMU.vy= get_int16(&dataBuf[22]);
    hedge->fusionIMU.vz= get_int16(&dataBuf[24]);
    
    hedge->fusionIMU.ax= get_int16(&dataBuf[26]);
    hedge->fusionIMU.ay= get_int16(&dataBuf[28]);
    hedge->fusionIMU.az= get_int16(&dataBuf[30]);

    hedge->fusionIMU.timestamp= get_uint32(&dataBuf[34]);
    
    hedge->fusionIMU.updated= true;
}

static void process_raw_distances_datagram(struct MarvelmindHedge * hedge, uint8_t *buffer)
{uint8_t *dataBuf= &buffer[5];
 uint8_t ofs;	
	
    hedge->rawDistances.address_hedge= dataBuf[0];
    
    ofs= 1;
    for(uint8_t i=0;i<4;i++)
    {
	   hedge->rawDistances.distances[i].address_beacon= dataBuf[ofs+0];	
	   hedge->rawDistances.distances[i].distance= get_uint32(&dataBuf[ofs+1]);
	   ofs+= 6;
	}
    
    hedge->rawDistances.updated= true;
}

////////////////////

enum
{
    RECV_HDR,
    RECV_DGRAM
};

//////////////////////////////////////////////////////////////////////////////
// Thread function started by MarvelmindHedge_start
//////////////////////////////////////////////////////////////////////////////

void
#ifndef WIN32
*
#endif // WIN32
Marvelmind_Thread_ (void* param)
{
    struct MarvelmindHedge * hedge=(struct MarvelmindHedge*) param;
    struct PositionValue curPosition;
    uint8_t input_buffer[256];
    uint8_t recvState=RECV_HDR; // current state of receive data
    uint8_t nBytesInBlockReceived=0; // bytes received
    uint16_t dataId;
 #ifndef WIN32
    struct pollfd fds[1];
    int pollrc;
 #endif

    SERIAL_PORT_HANDLE ttyHandle=OpenSerialPort_(hedge->ttyFileName,
                                 hedge->baudRate, hedge->verbose);
    if (ttyHandle==PORT_NOT_OPENED) hedge->terminationRequired=true;
    else if (hedge->verbose) printf ("Opened serial port %s with baudrate %u\n",
                                         hedge->ttyFileName, hedge->baudRate);

    while (hedge->terminationRequired==false)
    {
        uint8_t receivedChar;
        bool readSuccessed=true;
#ifdef WIN32
        DWORD nBytesRead;
        readSuccessed=ReadFile (ttyHandle, &receivedChar, 1, &nBytesRead, NULL);
#else
        int32_t nBytesRead;
        fds[0].fd = ttyHandle;
        fds[0].events = POLLIN ;
        pollrc = poll( fds, 1, 1000);
        if (pollrc<=0) continue;
        if ((fds[0].revents & POLLIN )==0) continue;      
        
        nBytesRead=read(ttyHandle, &receivedChar, 1);
        if (nBytesRead<0) readSuccessed=false;
#endif
        if (nBytesRead && readSuccessed)
        {
            bool goodByte= false;
            input_buffer[nBytesInBlockReceived]= receivedChar;
            switch (recvState)
            {
            case RECV_HDR:
                switch(nBytesInBlockReceived)
                {
                    case 0:
                        goodByte= (receivedChar == 0xff);
                        break;
                    case 1:
                        goodByte= (receivedChar == 0x47);
                        break;
                    case 2:
                        goodByte= true;
                        break;
                    case 3:
                        dataId= (((uint16_t) receivedChar)<<8) + input_buffer[2];
                        goodByte=   (dataId == POSITION_DATAGRAM_ID) ||
                                    (dataId == BEACONS_POSITIONS_DATAGRAM_ID) ||
                                    (dataId == POSITION_DATAGRAM_HIGHRES_ID) ||
                                    (dataId == BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID) ||
                                    (dataId == IMU_RAW_DATAGRAM_ID) ||
                                    (dataId == IMU_FUSION_DATAGRAM_ID) ||
                                    (dataId == BEACON_RAW_DISTANCE_DATAGRAM_ID);
                        break;
                    case 4:
                        switch(dataId )
                        {
                            case POSITION_DATAGRAM_ID:
                                goodByte= (receivedChar == 0x10);
                                break;
                            case BEACONS_POSITIONS_DATAGRAM_ID:
                            case BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID:
                                goodByte= true;
                                break;
                            case POSITION_DATAGRAM_HIGHRES_ID:
                                goodByte= (receivedChar == 0x16);
                                break;
                            case IMU_RAW_DATAGRAM_ID:
								goodByte= (receivedChar == 0x20);
                                break;
                            case IMU_FUSION_DATAGRAM_ID:
                                goodByte= (receivedChar == 0x2a);
                                break;
                            case BEACON_RAW_DISTANCE_DATAGRAM_ID:
                                goodByte= (receivedChar == 0x20);
                                break;
                        }
                        if (goodByte)
                            recvState=RECV_DGRAM;
                        break;
                }
                if (goodByte)
                {
                    // correct header byte
                    nBytesInBlockReceived++;
                }
                else
                {
                    // ...or incorrect
                    recvState=RECV_HDR;
                    nBytesInBlockReceived=0;
                }
                break;
            case RECV_DGRAM:
                nBytesInBlockReceived++;
                if (nBytesInBlockReceived>=7+input_buffer[4])
                {
                    // parse dgram
                    uint16_t blockCrc=
                        CalcCrcModbus_(input_buffer,nBytesInBlockReceived);
                    if (blockCrc==0)
                    {
#ifdef WIN32
                        EnterCriticalSection(&hedge->lock_);
#else
                        pthread_mutex_lock (&hedge->lock_);
#endif
                        switch(dataId )
                        {
                            case POSITION_DATAGRAM_ID:
                                // add to positionBuffer
                                curPosition= process_position_datagram(hedge, input_buffer);
                                break;
                            case BEACONS_POSITIONS_DATAGRAM_ID:
                                process_beacons_positions_datagram(hedge, input_buffer);
                                break;
                            case POSITION_DATAGRAM_HIGHRES_ID:
                                // add to positionBuffer
                                curPosition= process_position_highres_datagram(hedge, input_buffer);
                                break;
                            case BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID:
                                process_beacons_positions_highres_datagram(hedge, input_buffer);
                                break;
                            case IMU_RAW_DATAGRAM_ID:
								process_imu_raw_datagram(hedge, input_buffer);
                                break;
                            case IMU_FUSION_DATAGRAM_ID:
                                process_imu_fusion_datagram(hedge, input_buffer);
                                break;
                            case BEACON_RAW_DISTANCE_DATAGRAM_ID:
                                process_raw_distances_datagram(hedge, input_buffer);
                                break;
                        }
#ifdef WIN32
                        LeaveCriticalSection(&hedge->lock_);
#else
                        pthread_mutex_unlock (&hedge->lock_);
#endif
                        // callback
                        
                        if (hedge->anyInputPacketCallback)
                        {
							hedge->anyInputPacketCallback();
						}
						
                        if (hedge->receiveDataCallback)
                        {
                            if (dataId == POSITION_DATAGRAM_ID)
                            {
                                hedge->receiveDataCallback (curPosition);
                            }
                        }
                    }
                    // and repeat
                    recvState=RECV_HDR;
                    nBytesInBlockReceived=0;
                }
            }
        }
    }
#ifndef WIN32
    return NULL;
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Create an initialize MarvelmindHedge structure
// returncode: pointer to structure on success or NULL on error
//////////////////////////////////////////////////////////////////////////////
struct MarvelmindHedge * createMarvelmindHedge ()
{
    struct MarvelmindHedge * hedge=malloc (sizeof (struct MarvelmindHedge));
    if (hedge)
    {
        hedge->ttyFileName=DEFAULT_TTY_FILENAME;
        hedge->baudRate=9600;
        hedge->maxBufferedPositions=1;
        hedge->positionBuffer=NULL;
        hedge->verbose=false;
        hedge->receiveDataCallback=NULL;
        hedge->anyInputPacketCallback= NULL;
        hedge->lastValuesCount_=0;
        hedge->lastValues_next= 0;
        hedge->haveNewValues_=false;
        hedge->terminationRequired= false;
        
        hedge->rawIMU.updated= false;
        hedge->fusionIMU.updated= false;
        hedge->rawDistances.updated= false;
#ifdef WIN32
        InitializeCriticalSection(&hedge->lock_);
#else
        pthread_mutex_init (&hedge->lock_, NULL);
#endif
    }
    else puts ("Not enough memory");
    return hedge;
}

//////////////////////////////////////////////////////////////////////////////
// Initialize and start work thread
//////////////////////////////////////////////////////////////////////////////
void startMarvelmindHedge (struct MarvelmindHedge * hedge)
{uint8_t i;
	
    hedge->positionBuffer=
        malloc(sizeof (struct PositionValue)*hedge->maxBufferedPositions);
    if (hedge->positionBuffer==NULL)
    {
        if (hedge->verbose) puts ("Not enough memory");
        hedge->terminationRequired=true;
        return;
    }
    for(i=0;i<hedge->maxBufferedPositions;i++)
    {
        hedge->positionBuffer[i].ready= false;
        hedge->positionBuffer[i].processed= false;
    }
    for(i=0;i<MAX_STATIONARY_BEACONS;i++)
    {
        hedge->positionsBeacons.beacons[i].updatedForMsg= false;
    }
    hedge->positionsBeacons.numBeacons= 0;
    hedge->positionsBeacons.updated= false;
    
#ifdef WIN32
    _beginthread (Marvelmind_Thread_, 0, hedge);
#else
    pthread_create (&hedge->thread_, NULL, Marvelmind_Thread_, hedge);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Write average position coordinates
// hedge:      MarvelmindHedge structure
// position:   pointer to PositionValue for write coordinates
// returncode: true if position is valid
//////////////////////////////////////////////////////////////////////////////

static bool getPositionFromMarvelmindHedgeByAddress (struct MarvelmindHedge * hedge,
                                     struct PositionValue * position, uint8_t address)
{
    uint8_t i;
    int32_t avg_x=0, avg_y=0, avg_z=0;
    double avg_ang= 0.0;
    uint32_t max_timestamp=0;
    bool position_valid;
    bool highRes= false;
    uint8_t flags= 0;
#ifdef WIN32
    EnterCriticalSection(&hedge->lock_);
#else
    pthread_mutex_lock (&hedge->lock_);
#endif
    if (hedge->lastValuesCount_)
    {
        uint8_t real_values_count=hedge->maxBufferedPositions;
        uint8_t nFound= 0;
        if (hedge->lastValuesCount_<real_values_count)
            real_values_count=hedge->lastValuesCount_;
        for (i=0; i<real_values_count; i++)
        {
            if (address != 0)
                if (hedge->positionBuffer[i].address != address)
                    continue;
            if (!hedge->positionBuffer[i].ready)
                continue;
            if (hedge->positionBuffer[i].processed)
                continue;
            if (address == 0)
                address= hedge->positionBuffer[i].address;
            nFound++;
            avg_x+=hedge->positionBuffer[i].x;
            avg_y+=hedge->positionBuffer[i].y;
            avg_z+=hedge->positionBuffer[i].z;
            avg_ang+= hedge->positionBuffer[i].angle;
            
            flags= hedge->positionBuffer[i].flags;
            if (flags&(1<<0)) 
              {
				position_valid=false;
			  }
            
            if (hedge->positionBuffer[i].highResolution)
                highRes= true;
            hedge->positionBuffer[i].processed= true;
            if (hedge->positionBuffer[i].timestamp>max_timestamp)
                max_timestamp=hedge->positionBuffer[i].timestamp;
        }
        if (nFound != 0)
        {
            avg_x/=nFound;
            avg_y/=nFound;
            avg_z/=nFound;
            avg_ang/=nFound;
            position_valid=true;
        } else
        {
            position_valid=false;
        }
    }
    else position_valid=false;
#ifdef WIN32
    LeaveCriticalSection(&hedge->lock_);
#else
    pthread_mutex_unlock (&hedge->lock_);
#endif
    if (!position_valid) 
    {
		flags|= (1<<0);// coordiantes not available
	}
    position->address= address;
    position->x=avg_x;
    position->y=avg_y;
    position->z=avg_z;
    position->angle= avg_ang;
    position->timestamp=max_timestamp;
    position->ready= position_valid;
    position->highResolution= highRes;
    position->flags= flags;
    return position_valid;
}

bool getPositionFromMarvelmindHedge (struct MarvelmindHedge * hedge,
                                     struct PositionValue * position)
{
    return getPositionFromMarvelmindHedgeByAddress(hedge, position, 0);
};

//////////////////////////////////////////////////////////////////////////////
// Print average position coordinates
// onlyNew: print only new positions
//////////////////////////////////////////////////////////////////////////////
void printPositionFromMarvelmindHedge (struct MarvelmindHedge * hedge,
    bool onlyNew)
{uint8_t i,j;
 double xm,ym,zm;

    if (hedge->haveNewValues_ || (!onlyNew))
    {
        struct PositionValue position;
        uint8_t real_values_count=hedge->maxBufferedPositions;
        uint8_t addresses[real_values_count];
        uint8_t addressesNum= 0;

        for(i=0;i<real_values_count;i++)
        {
           uint8_t address= hedge->positionBuffer[i].address;
           bool alreadyProcessed= false;
           if (addressesNum != 0)
                for(j=0;j<addressesNum;j++)
                {
                    if (address == addresses[j])
                    {
                        alreadyProcessed= true;
                        break;
                    }
               }
            if (alreadyProcessed)
                continue;
            addresses[addressesNum++]= address;

            getPositionFromMarvelmindHedgeByAddress (hedge, &position, address);
            xm= ((double) position.x)/1000.0;
            ym= ((double) position.y)/1000.0;
            zm= ((double) position.z)/1000.0;
            if (position.ready)
            {
                if (position.highResolution)
                {
                    printf ("Address: %d, X: %.3f, Y: %.3f, Z: %.3f at time T: %u\n",
                            position.address, xm, ym, zm, position.timestamp);
                } else
                {
                    printf ("Address: %d, X: %.2f, Y: %.2f, Z: %.2f at time T: %u\n",
                            position.address, xm, ym, zm, position.timestamp);
                }
            }
            hedge->haveNewValues_=false;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
// Get positions of stationary beacons
// hedge:       MarvelmindHedge structure
// positions:   pointer to structure for write coordinates
//////////////////////////////////////////////////////////////////////////////

bool getStationaryBeaconsPositionsFromMarvelmindHedge (struct MarvelmindHedge * hedge,
                                              struct StationaryBeaconsPositions * positions)
{
#ifdef WIN32
    EnterCriticalSection(&hedge->lock_);
#else
    pthread_mutex_lock (&hedge->lock_);
#endif

    *positions= hedge->positionsBeacons;

#ifdef WIN32
    LeaveCriticalSection(&hedge->lock_);
#else
    pthread_mutex_unlock (&hedge->lock_);
#endif

    return true;
}

void clearStationaryBeaconUpdatedFlag(struct MarvelmindHedge * hedge, uint8_t address)
{uint8_t i,n;
	
#ifdef WIN32
    EnterCriticalSection(&hedge->lock_);
#else
    pthread_mutex_lock (&hedge->lock_);
#endif

	n= hedge->positionsBeacons.numBeacons;
	
	for(i=0;i<n;i++)
	{
		if (hedge->positionsBeacons.beacons[i].address == address)
		{
			hedge->positionsBeacons.beacons[i].updatedForMsg= false;
		}
	}
	
#ifdef WIN32
    LeaveCriticalSection(&hedge->lock_);
#else
    pthread_mutex_unlock (&hedge->lock_);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Print stationary beacons positions
// onlyNew: print only new positions
//////////////////////////////////////////////////////////////////////////////
void printStationaryBeaconsPositionsFromMarvelmindHedge (struct MarvelmindHedge * hedge,
                                                         bool onlyNew)
{struct StationaryBeaconsPositions positions;
  double xm,ym,zm;

    getStationaryBeaconsPositionsFromMarvelmindHedge(hedge, &positions);

    if (positions.updated || (!onlyNew))
    {uint8_t i;
     uint8_t n= hedge->positionsBeacons.numBeacons;
     struct StationaryBeaconPosition *b;

        for(i=0;i<n;i++)
        {
            b= &positions.beacons[i];
            xm= ((double) b->x)/1000.0;
            ym= ((double) b->y)/1000.0;
            zm= ((double) b->z)/1000.0;
            if (positions.beacons[i].highResolution)
            {
                printf ("Stationary beacon: address: %d, X: %.3f, Y: %.3f, Z: %.3f \n",
                            b->address,xm, ym, zm);
            } else
            {
                printf ("Stationary beacon: address: %d, X: %.2f, Y: %.2f, Z: %.2f \n",
                            b->address,xm, ym, zm);
            }
        }

        hedge->positionsBeacons.updated= false;
    }
}

//////////////////////////////////////////////////////////////////////////////
// Stop work thread
//////////////////////////////////////////////////////////////////////////////
void stopMarvelmindHedge (struct MarvelmindHedge * hedge)
{
    hedge->terminationRequired=true;
    if (hedge->verbose) puts ("stopping");
#ifdef WIN32
    WaitForSingleObject (hedge->thread_, INFINITE);
#else
    pthread_join (hedge->thread_, NULL);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Destroy structures to free memory (You must call stopMarvelmindHedge
// first)
//////////////////////////////////////////////////////////////////////////////
void destroyMarvelmindHedge (struct MarvelmindHedge * hedge)
{
    if (hedge->positionBuffer) free (hedge->positionBuffer);
    free (hedge);
}
