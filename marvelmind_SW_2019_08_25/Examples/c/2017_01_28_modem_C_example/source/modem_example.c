#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <signal.h>
#endif // WIN32
#include "marvelmind_modem.h"

bool terminateProgram=false;

uint32_t requestRepeatsCount= 1;

#define CMD_STATE 0
#define CMD_SLEEP 1
#define CMD_DEEPSLEEP 2
#define CMD_WAKE 3

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
void sleepMs(unsigned int ms)
{
    Sleep (ms);
}
#else
void sleepMs(unsigned int ms)
{
    usleep (ms*1000);
}
#endif

void processBeaconState(void *data)
{struct BeaconState *beaconState= (struct BeaconState *) data;

    if (beaconState->rq.resultCode == MM_EXCHANGE_OK)
    {
        printf ("Beacon %d state: RSSI= %d, Vcc= %.3f, Time= %ld \n",
               (int) beaconState->rq.address, (int) beaconState->RSSI_dbM,
                ((double) beaconState->VCC_mV) /1000.0, (long int) beaconState->workingTimeSeconds);
    }
    else
    {
        printf ("Failed read beacon %d state: error %d \n",
               (int) beaconState->rq.address,(int) beaconState->rq.resultCode);
    }

    if (requestRepeatsCount>0)
        requestRepeatsCount--;
}

void processSleepControlCmdReply(void *data)
{struct BeaconSleepControl *beaconSleepControl= (struct BeaconSleepControl *) data;

    if (beaconSleepControl->rq.resultCode == MM_EXCHANGE_OK)
    {
        printf ("Beacon sleep control command %d to device %d completed \n",
               (int) beaconSleepControl->sleepControlCmd, (int) beaconSleepControl->rq.address);
    }
    else
    {
        printf ("Beacon sleep control command %d for device %d failed: error %d \n",
               (int) beaconSleepControl->sleepControlCmd, (int) beaconSleepControl->rq.address,(int) beaconSleepControl->rq.resultCode);
    }

    if (requestRepeatsCount>0)
        requestRepeatsCount--;
}

static void trySendCommand(struct MarvelmindModem * modem, uint8_t cmd, uint8_t address)
{
    if (requestRepeatsCount == 0)
        return;

    switch(cmd)
    {
    case CMD_STATE:
        {
            if (modem->beaconState.rq.state != inProgress)
            {
                startGetBeaconState(modem,address,&processBeaconState);
            }
            break;
        }

    case CMD_SLEEP:
    case CMD_DEEPSLEEP:
    case CMD_WAKE:
        {uint8_t sleepCmd;

            if (modem->beaconState.rq.state != inProgress)
            {
                switch(cmd)
                {
                    case CMD_SLEEP: {sleepCmd= SCC_SLEEP_NORMAL; break;};
                    case CMD_DEEPSLEEP: {sleepCmd= SCC_SLEEP_DEEP; break;};
                    case CMD_WAKE: {sleepCmd= SCC_WAKE; break;};
                }
                startSendSleepControlCmd(modem, address, sleepCmd, &processSleepControlCmdReply);
            }
            break;
        }
    }
}

int main (int argc, char *argv[])
{int i;
    // process command line
    const char * ttyFileName= DEFAULT_TTY_FILENAME;
    uint8_t beaconAddress= 2;
    char *curArg;
    uint8_t cmd= CMD_STATE;
    if (argc>1)
        for(i=1;i<argc;i++)
        {
            curArg= argv[i];

            if (curArg[0] == 0) continue;
            if (strcmp(curArg,"state") == 0) {
                cmd= CMD_STATE;
                continue;
            }
            else if (strcmp(curArg,"sleep") == 0) {
                cmd= CMD_SLEEP;
                continue;
            }
            else if (strcmp(curArg,"deepsleep") == 0) {
                cmd= CMD_DEEPSLEEP;
                continue;
            }
            else if (strcmp(curArg,"wake") == 0) {
                cmd= CMD_WAKE;
                continue;
            }
            else if ((curArg[0]== 'n'))
            {
                beaconAddress= atoi(&curArg[1]);
                continue;
            }
            else if ((curArg[0]>='0')&&(curArg[0]<='9'))
            {
                requestRepeatsCount= atoi(&curArg[0]);
                continue;
            }
            else
            {
               ttyFileName=argv[1];
               continue;
            }
        }

    // Init
    struct MarvelmindModem * modem=createMarvelmindModem ();
    if (modem==NULL)
    {
        puts ("Error: Unable to create MarvelmindModem");
        return -1;
    }
    modem->ttyFileName=ttyFileName;
    modem->verbose=true; // show errors and warnings
    startMarvelmindModem (modem);

    // Set Ctrl-C handler
#ifdef WIN32
    SetConsoleCtrlHandler( (PHANDLER_ROUTINE) CtrlHandler, TRUE );
#else
    signal (SIGINT, CtrlHandler);
    signal (SIGQUIT, CtrlHandler);
#endif

    trySendCommand(modem, cmd, beaconAddress);
    // Main loop
    while ((!terminateProgram) && (!modem->terminationRequired))
    {

        trySendCommand(modem, cmd, beaconAddress);

        if (requestRepeatsCount == 0)
        {
            modem->terminationRequired= true;
        }

        sleepMs (500);
    }

    // Exit
    stopMarvelmindModem (modem);
    destroyMarvelmindModem (modem);
    return 0;
}

