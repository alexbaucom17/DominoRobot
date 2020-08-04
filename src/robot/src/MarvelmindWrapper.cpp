#include "MarvelmindWrapper.h"
#include "constants.h"
#include "plog/Log.h"

extern "C" {
    #include <marvelmind/marvelmind_api.h>
}

MarvelmindWrapper::MarvelmindWrapper()
{
    marvelmindAPILoad();
    PLOGI << "Marvelmind API loaded";

    // I expect to connect to 2 devices, but because of the fun of linux and udev rules, I don't know 
    // which of the three possible names the devices could have. I could later change this to first 
    // do an `ls /dev/ | grep marvelmind` and find the right ports, but this should work fine too.
    char portName0[64] = MARVELMIND_USB_0;
    char portName1[64] = MARVELMIND_USB_1;
    char portName2[64] = MARVELMIND_USB_2;
    bool success0 = mmOpenPortByName(portName0);
    bool success1 = mmOpenPortByName(portName1);
    bool success2 = mmOpenPortByName(portName2);
    int success = static_cast<int>(success0) + static_cast<int>(success1) + static_cast<int>(success2);
    if(success == 2)
    {
        PLOGI << "Successfully connected to 2 Marvelmind devices";
    }
    else if (success == 1)
    {
        PLOGW << "Only able to connect to 1 Marvelmind device";
    }
    else if (success == 0)
    {
        PLOGW << "Unable to connect to any Marvelmind devices";
    }
    else
    {
        PLOGE << "Managed to connect to " << success << "Marvelmind devices. That shouldn't happen!";
    }
}

MarvelmindWrapper::~MarvelmindWrapper()
{
    marvelmindAPIFree();
}