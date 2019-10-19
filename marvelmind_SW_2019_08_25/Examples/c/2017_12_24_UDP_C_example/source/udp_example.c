#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#ifdef WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#endif // WIN32
#include "udp_marvelmind.h"

bool terminateProgram=false;

#ifdef WIN32
BOOL CtrlHandler( DWORD fdwCtrlType )
{
    if ((fdwCtrlType==CTRL_C_EVENT ||
            fdwCtrlType==CTRL_BREAK_EVENT ||
            fdwCtrlType==CTRL_CLOSE_EVENT ||
            fdwCtrlType==CTRL_LOGOFF_EVENT ||
            fdwCtrlType==CTRL_SHUTDOWN_EVENT) &&
            (terminateProgram==false))
    {
        terminateProgram=true;
        return true;
    }
    else return false;
}
#else
void CtrlHandler(int signum)
{
    terminateProgram=true;
}
#endif

#ifdef WIN32
// Windows
HANDLE ghSemaphore;
DWORD dwSemWaitResult;
void semCallback()
{
    ReleaseSemaphore(
        ghSemaphore,  // handle to semaphore
        1,            // increase count by one
        NULL);
}
#else
// Linux
static sem_t *sem;
struct timespec ts;
void semCallback()
{
	sem_post(sem);
}
#endif // WIN32

int main (int argc, char *argv[])
{
    uint8_t beaconRequestAddress;
    if (argc>=2) beaconRequestAddress=atoi(argv[1]);
    else beaconRequestAddress=0;

    // get server address from command line
    const char * serverAddress;
    if (argc>=3) serverAddress=argv[2];
    else serverAddress=DEFAULT_UDP_SERVER_ADDRESS;

    uint16_t serverPort;
    if (argc>=4) serverPort=atoi(argv[3]);
    else serverPort=DEFAULT_UDP_SERVER_PORT;

    // Init
    struct MarvelmindUDP * udp=createMarvelmindUDP ();
    if (udp==NULL)
    {
        puts ("Error: Unable to create MarvelmindUDP");
        return -1;
    }
    udp->serverAddress=serverAddress;
    udp->serverPort= serverPort;
    udp->beaconRequestAddress= beaconRequestAddress;
    udp->requestRateHz= 16;
    udp->verbose=true; // show errors and warnings
    udp->anyInputPacketCallback= semCallback;
    startMarvelmindUDP (udp);

    // Set Ctrl-C handler
#ifdef WIN32
    SetConsoleCtrlHandler( (PHANDLER_ROUTINE) CtrlHandler, TRUE );
#else
    signal (SIGINT, CtrlHandler);
    signal (SIGQUIT, CtrlHandler);
#endif

#ifdef WIN32
	// Windows
    ghSemaphore = CreateSemaphore(
        NULL, // default security attributes
        10,  // initial count
        10,  // maximum count
        NULL);          // unnamed semaphore
    if (ghSemaphore == NULL)
    {
        printf("CreateSemaphore error: %d\n", GetLastError());
        return 1;
    }
#else
	// Linux
	sem = sem_open(DATA_INPUT_SEMAPHORE, O_CREAT, 0777, 0);
#endif

    // Main loop
    while ((!terminateProgram) && (!udp->terminationRequired))
    {
         #ifdef WIN32
         // Windows
        dwSemWaitResult = WaitForSingleObject(
            ghSemaphore,   // handle to semaphore
            1000); // time-out interval
        #else
        // Linux
        if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
		{
			printf("clock_gettime error");
			return -1;
		}
		ts.tv_sec += 2;
		sem_timedwait(sem,&ts);
        #endif

        printPositionFromMarvelmindUDP (udp, true);
    }

    // Exit
    stopMarvelmindUDP (udp);
    destroyMarvelmindUDP (udp);
    return 0;
}
