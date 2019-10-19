#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#ifdef WIN32
#include <windows.h>
#include <process.h>
#include <winsock2.h>
#else
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#endif // WIN32
#include "udp_marvelmind.h"

#ifdef WIN32
#pragma comment(lib,"ws2_32.lib") //Winsock Library
#endif

#define UDP_BUFFER_SIZE 255

#ifndef WIN32
#define SOCKET_ERROR (-1)
#endif

#define FRAME_ID_POSITION_CM 0x0001
#define FRAME_ID_POSITION_MM 0x0011

static uint8_t prepareUDPRequest(uint8_t beaconAddress, uint16_t dataCode, uint8_t bytesToRead, uint8_t *buf)
{
    buf[0]= beaconAddress;

    buf[1]= 0x47;

    buf[2]= dataCode&0xff;
    buf[3]= (dataCode>>8)&0xff;

    buf[4]= 0x04;

    buf[5]= 0x00;
    buf[6]= 0x00;

    buf[7]= bytesToRead;

    buf[8]= 0;
    buf[9]= 0;// CRC not required

    return 10;
}

//////////////////////////////////////////////////////////////////////////////
// Thread function started by MarvelmindUDP_start
//////////////////////////////////////////////////////////////////////////////
void
#ifndef WIN32
*
#endif // WIN32
Marvelmind_Thread_ (void* param)
{
    struct MarvelmindUDP * udp=(struct MarvelmindUDP*) param;

    uint8_t lastValues_next=0;
    int s;
    struct sockaddr_in si_other, si_me;
    int slen = sizeof(si_other) ;
    uint8_t bufferInput[UDP_BUFFER_SIZE];
    uint8_t bufferOutput[UDP_BUFFER_SIZE];
    uint8_t dataSize;
    bool highRes;
    uni_8x2_16 v16;
    uni_8x4_32 v32;

#ifdef WIN32
    WSADATA wsa;
    //Initialise winsock
    if (WSAStartup(MAKEWORD(2,2),&wsa) != 0)
    {
        printf("Winsock initialization failed. Error Code : %d\n",WSAGetLastError());
        return;
    }
#endif

    //create socket
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
    {
#ifdef WIN32
        printf("socket() failed with error code: %d \n" , WSAGetLastError());
        return;
#else
        printf("socket() failed\n");
        return NULL;
#endif // WIN32
    }
    printf("Socket opened\n");
    //setup address structure
    memset((char *) &si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(udp->serverPort);
#ifdef WIN32
	si_other.sin_addr.S_un.S_addr = inet_addr(udp->serverAddress);
#else
	si_other.sin_addr.s_addr = inet_addr(udp->serverAddress);
#endif
    if (udp->beaconRequestAddress == 0)
    {// want to listen for stream
        si_me.sin_family = AF_INET;
        si_me.sin_port = htons(udp->serverPort);
#ifdef WIN32
        si_me.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
#else
        si_me.sin_addr.s_addr = htonl(INADDR_ANY);
#endif
        if (bind(s, (struct sockaddr *)&si_me, sizeof(si_me)) == SOCKET_ERROR)
        {
#ifdef WIN32
            printf("bind() failed: %d.\n", WSAGetLastError());
            closesocket(s);
            return;
#else
            return NULL;
#endif
        }
    }

    printf("UDP opened: address= %s   port= %d \n" , udp->serverAddress, udp->serverPort);

    int failCount= 0;
    while (udp->terminationRequired==false)
    {
#ifdef WIN32
        EnterCriticalSection(&udp->lock_);
#else
        pthread_mutex_lock (&udp->lock_);
#endif
        int sleepTimeMs;
        if (udp->requestRateHz == 0)
        {
            sleepTimeMs= 1000;
        }
        else
        {
            sleepTimeMs= (1000/udp->requestRateHz);
        }

        uint8_t rqAddress= udp->beaconRequestAddress;
#ifdef WIN32
        LeaveCriticalSection(&udp->lock_);
#else
        pthread_mutex_unlock (&udp->lock_);
#endif
        int sleepTimeFail= (failCount+1)*10;
        if (sleepTimeFail>sleepTimeMs)
            sleepTimeMs= sleepTimeFail;

        if (rqAddress != 0) {
            if (udp->requestRateHz != 0)
            {
#ifdef WIN32
                Sleep(sleepTimeMs);
#else
                usleep(sleepTimeMs*1000);
#endif // WIN32
            }

            int sendSize= prepareUDPRequest(rqAddress, FRAME_ID_POSITION_MM, 0x16, bufferOutput);
            //send the message
            if (sendto(s, (const char *) bufferOutput, sendSize , 0 , (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
            {
#ifdef WIN32
                printf("sendto() failed with error code : %d\n" , WSAGetLastError());
#endif // WIN32
                if (failCount<100)
                    failCount++;
                continue;
            }
        }

        int recvSize= 29;
        //try to receive some data, this is a blocking call
        if (recvfrom(s, (char *) bufferInput, recvSize, 0, (struct sockaddr *) &si_other, &slen) == SOCKET_ERROR)
        {
            if (failCount<100)
                failCount++;
            continue;
        }
        failCount= 0;

        if (bufferInput[1] != 0x47)
            continue;
        uint16_t dataCode= bufferInput[2] + (((int) bufferInput[3])<<8);
        switch(dataCode)
        {
          case FRAME_ID_POSITION_MM:
            {
               dataSize= 0x16;
               highRes= true;
               break;
            }

          case FRAME_ID_POSITION_CM:
            {
               dataSize= 0x10;
               highRes= false;
               break;
            }

          default:
            {
              continue;
            }
        }
        if (bufferInput[4] != dataSize)
            continue;

        uint8_t addressReceived= bufferInput[0];
        v32.b[0]= bufferInput[5];
        v32.b[1]= bufferInput[6];
        v32.b[2]= bufferInput[7];
        v32.b[3]= bufferInput[8];
        uint32_t timestamp= v32.dw;

        int32_t xc,yc,zc;
        if (highRes)
        {
           v32.b[0]= bufferInput[9];
           v32.b[1]= bufferInput[10];
           v32.b[2]= bufferInput[11];
           v32.b[3]= bufferInput[12];
           xc= v32.dwi;

           v32.b[0]= bufferInput[13];
           v32.b[1]= bufferInput[14];
           v32.b[2]= bufferInput[15];
           v32.b[3]= bufferInput[16];
           yc= v32.dwi;

           v32.b[0]= bufferInput[17];
           v32.b[1]= bufferInput[18];
           v32.b[2]= bufferInput[19];
           v32.b[3]= bufferInput[20];
           zc= v32.dwi;
        }
        else
        {
           v16.b[0]= bufferInput[9];
           v16.b[1]= bufferInput[10];
           xc= ((int32_t) v16.wi)*10;// millimeters

           v16.b[0]= bufferInput[11];
           v16.b[1]= bufferInput[12];
           yc= ((int32_t) v16.wi)*10;// millimeters

           v16.b[0]= bufferInput[13];
           v16.b[1]= bufferInput[14];
           zc= ((int32_t) v16.wi)*10;// millimeters
        }

        // add to positionBuffer
#ifdef WIN32
        EnterCriticalSection(&udp->lock_);
#else
        pthread_mutex_lock (&udp->lock_);
#endif
        udp->positionBuffer[lastValues_next].address= addressReceived;
        udp->positionBuffer[lastValues_next].timestamp= timestamp;
        udp->positionBuffer[lastValues_next].x= xc;
        udp->positionBuffer[lastValues_next].y= yc;
        udp->positionBuffer[lastValues_next].z= zc;
        udp->positionBuffer[lastValues_next].ready= true;

        lastValues_next++;
        if (lastValues_next>=udp->maxBufferedPositions)
            lastValues_next=0;
        if (udp->lastValuesCount_<udp->maxBufferedPositions)
            udp->lastValuesCount_++;
        udp->haveNewValues_=true;
#ifdef WIN32
        LeaveCriticalSection(&udp->lock_);
#else
        pthread_mutex_unlock (&udp->lock_);
#endif
        // callback
        if (udp->anyInputPacketCallback) {
            udp->anyInputPacketCallback();
        }

        if (udp->receiveDataCallback)
        {
            struct PositionValue position=
            {
                addressReceived,
                timestamp,
                xc,
                yc,
                zc
            };
            udp->receiveDataCallback (position);
        }
    }
#ifndef WIN32
    return NULL;
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Create an initialize MarvelmindUDP structure
// returncode: pointer to structure on success or NULL on error
//////////////////////////////////////////////////////////////////////////////
struct MarvelmindUDP * createMarvelmindUDP ()
{
    struct MarvelmindUDP * udp=malloc (sizeof (struct MarvelmindUDP));
    if (udp)
    {
        udp->serverAddress=DEFAULT_UDP_SERVER_ADDRESS;
        udp->serverPort=DEFAULT_UDP_SERVER_PORT;
        udp->beaconRequestAddress= 0;
        udp->requestRateHz= 16;
        udp->maxBufferedPositions=3;
        udp->positionBuffer=NULL;
        udp->verbose=false;
        udp->receiveDataCallback=NULL;
        udp->anyInputPacketCallback= NULL;
        udp->lastValuesCount_=0;
        udp->haveNewValues_=false;
        udp->terminationRequired= false;
#ifdef WIN32
        InitializeCriticalSection(&udp->lock_);
#else
        pthread_mutex_init (&udp->lock_, NULL);
#endif
    }
    else puts ("Not enough memory");
    return udp;
}

//////////////////////////////////////////////////////////////////////////////
// Initialize and start work thread
//////////////////////////////////////////////////////////////////////////////
void startMarvelmindUDP (struct MarvelmindUDP * udp)
{uint8_t i;
    udp->positionBuffer=
        malloc(sizeof (struct PositionValue)*udp->maxBufferedPositions);
    if (udp->positionBuffer==NULL)
    {
        if (udp->verbose) puts ("Not enough memory");
        udp->terminationRequired=true;
        return;
    }

    for(i=0;i<udp->maxBufferedPositions;i++)
        udp->positionBuffer[i].ready= false;

#ifdef WIN32
    _beginthread (Marvelmind_Thread_, 0, udp);
#else
    pthread_create (&udp->thread_, NULL, Marvelmind_Thread_, udp);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Write average position coordinates
// udp:        MarvelmindUDP structure
// address:    address of hedge to get position
// position:   pointer to PositionValue for write coordinates
// returncode: true if position is valid
//////////////////////////////////////////////////////////////////////////////
bool getPositionFromMarvelmindUDP (struct MarvelmindUDP * udp,
                                   uint8_t address,
                                   struct PositionValue * position)
{
    uint8_t i;
    int32_t avg_x=0, avg_y=0, avg_z=0;
    uint32_t max_timestamp=0;
    bool position_valid;
#ifdef WIN32
    EnterCriticalSection(&udp->lock_);
#else
    pthread_mutex_lock (&udp->lock_);
#endif
    if (udp->lastValuesCount_)
    {
        uint8_t real_values_count=udp->maxBufferedPositions;
        uint8_t nFound= 0;
        if (udp->lastValuesCount_<real_values_count)
            real_values_count=udp->lastValuesCount_;
        for (i=0; i<real_values_count; i++)
        {
            if (udp->positionBuffer[i].address != address)
                continue;
            if (!udp->positionBuffer[i].ready)
                continue;
            nFound++;
            avg_x+=udp->positionBuffer[i].x;
            avg_y+=udp->positionBuffer[i].y;
            avg_z+=udp->positionBuffer[i].z;
            if (udp->positionBuffer[i].timestamp>max_timestamp)
                max_timestamp=udp->positionBuffer[i].timestamp;
        }
        if (nFound != 0)
        {
          avg_x/=nFound;
          avg_y/=nFound;
          avg_z/=nFound;
          position_valid=true;
        } else
        {
          position_valid=false;
        }
    }
    else position_valid=false;
#ifdef WIN32
    LeaveCriticalSection(&udp->lock_);
#else
    pthread_mutex_unlock (&udp->lock_);
#endif
    position->address= address;
    position->x=avg_x;
    position->y=avg_y;
    position->z=avg_z;
    position->timestamp=max_timestamp;
    position->ready= position_valid;
    return position_valid;
}

//////////////////////////////////////////////////////////////////////////////
// Print average position coordinates
// onlyNew: print only new positions
//////////////////////////////////////////////////////////////////////////////
void printPositionFromMarvelmindUDP (struct MarvelmindUDP * udp, bool onlyNew)
{uint8_t i,j;

    if (udp->haveNewValues_ || (!onlyNew))
    {
        struct PositionValue position;
        uint8_t real_values_count=udp->maxBufferedPositions;
        uint8_t addresses[real_values_count];
        uint8_t addressesNum= 0;

        for(i=0;i<real_values_count;i++)
        {
           uint8_t address= udp->positionBuffer[i].address;
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

            getPositionFromMarvelmindUDP (udp, address, &position);
            if (position.ready)
                printf ("Address: %d, X: %d, Y: %d, Z: %d at time T: %u\n", address, position.x,
                        position.y, position.z, position.timestamp);
        }
        udp->haveNewValues_=false;
    }
}

//////////////////////////////////////////////////////////////////////////////
// Stop work thread
//////////////////////////////////////////////////////////////////////////////
void stopMarvelmindUDP (struct MarvelmindUDP * udp)
{
    udp->terminationRequired=true;
    if (udp->verbose) puts ("stopping");
#ifdef WIN32
    WaitForSingleObject (udp->thread_, INFINITE);
#else
    pthread_join (udp->thread_, NULL);
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Destroy structures to free memory (You must call stopMarvelmindUDP
// first)
//////////////////////////////////////////////////////////////////////////////
void destroyMarvelmindUDP (struct MarvelmindUDP * udp)
{
    if (udp->positionBuffer) free (udp->positionBuffer);
    free (udp);
}
