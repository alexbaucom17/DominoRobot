#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#ifdef WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <semaphore.h>
#include <time.h>
#endif // WIN32
#include "marvelmind.h"

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
void sleep(unsigned int seconds)
{
    Sleep (seconds*1000);
}
#endif


#ifdef WIN32
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
    // get port name from command line arguments (if specified)
    const char * ttyFileName;
    if (argc==2) ttyFileName=argv[1];
    else ttyFileName=DEFAULT_TTY_FILENAME;

    // Init
    struct MarvelmindHedge * hedge=createMarvelmindHedge ();
    if (hedge==NULL)
    {
        puts ("Error: Unable to create MarvelmindHedge");
        return -1;
    }
    hedge->ttyFileName=ttyFileName;
    hedge->verbose=true; // show errors and warnings
    hedge->anyInputPacketCallback= semCallback;
    startMarvelmindHedge (hedge);

    // Set Ctrl-C handler
#ifdef WIN32
    SetConsoleCtrlHandler( (PHANDLER_ROUTINE) CtrlHandler, TRUE );
#else
    signal (SIGINT, CtrlHandler);
    signal (SIGQUIT, CtrlHandler);
#endif

#ifdef WIN32
    ghSemaphore = CreateSemaphore(
        NULL, // default security attributes
        10,  // initial count
        10,  // maximum count
        NULL);          // unnamed semaphore
    if (ghSemaphore == NULL)
    {
        printf("CreateSemaphore error: %d\n", (int) GetLastError());
        return 1;
    }
#else
	// Linux
	sem = sem_open(DATA_INPUT_SEMAPHORE, O_CREAT, 0777, 0);
#endif

    // Main loop
    while ((!terminateProgram) && (!hedge->terminationRequired))
    {
        //sleep (3);
        #ifdef WIN32
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


        printPositionFromMarvelmindHedge (hedge, true);

        printStationaryBeaconsPositionsFromMarvelmindHedge (hedge, true);

        printRawDistancesFromMarvelmindHedge(hedge, true);

        printRawIMUFromMarvelmindHedge(hedge, true);

        printFusionIMUFromMarvelmindHedge(hedge, true);
    }

    // Exit
    stopMarvelmindHedge (hedge);
    destroyMarvelmindHedge (hedge);
    return 0;
}
