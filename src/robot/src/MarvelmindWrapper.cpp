#include "MarvelmindWrapper.h"
#include "constants.h"
#include "plog/Log.h"
#include <filesystem>


MarvelmindWrapper::MarvelmindWrapper()
: hedge_(createMarvelmindHedge()),
  ready_(false)
{
    // I expect to 2 devices connected, but because of the fun of linux and udev rules, I don't know 
    // which of the three possible names the devices could have. I could later change this to first 
    // do an `ls /dev/ | grep marvelmind` and find the right ports, but this should work fine too.
    if(std::filesystem::exists(MARVELMIND_USB_0))
    {
        hedge_->ttyFileName = MARVELMIND_USB_0;
        PLOGI << "Found device " << MARVELMIND_USB_0;
    }
    else if(std::filesystem::exists(MARVELMIND_USB_1))
    {
        hedge_->ttyFileName = MARVELMIND_USB_1;
        PLOGI << "Found device " << MARVELMIND_USB_1;
    }
    else if(std::filesystem::exists(MARVELMIND_USB_2))
    {
        hedge_->ttyFileName = MARVELMIND_USB_2;
        PLOGI << "Found device " << MARVELMIND_USB_2;
    }
    else
    {
        PLOGW << "No marvelmind devices connected";
        return;
    }

    // Now actually try to connect
    hedge_->verbose = true;
    startMarvelmindHedge(hedge_);
    ready_ = true;
}

std::vector<float> MarvelmindWrapper::getPositions()
{
    PositionValue position_data;
    std::vector<float> output;
    
    if (!ready_)
    {
        return output;
    }

    bool ok = getPositionFromMarvelmindHedge(hedge_, &position_data);

    if (ok)
    {
        //PLOGI.printf("Position data - ready: %i, addr: %i, ts: %i, x: %i, y: %i, a:%f", position_data.ready, position_data.address, position_data.timestamp, position_data.x, position_data.y, position_data.angle);

        if (position_data.ready)
        {
            output.push_back(position_data.x/1000.0);
            output.push_back(position_data.y/1000.0);
            output.push_back(position_data.angle);
        }
    }

    return output;
}


MarvelmindWrapper::~MarvelmindWrapper()
{
    stopMarvelmindHedge (hedge_);
    destroyMarvelmindHedge (hedge_);
}



// MarvelmindWrapper::MarvelmindWrapper()
// {
//     marvelmindAPILoad();
//     PLOGI << "Marvelmind API loaded";

//     // I expect to connect to 2 devices, but because of the fun of linux and udev rules, I don't know 
//     // which of the three possible names the devices could have. I could later change this to first 
//     // do an `ls /dev/ | grep marvelmind` and find the right ports, but this should work fine too.
//     // char portName0[64] = MARVELMIND_USB_0;
//     // char portName1[64] = MARVELMIND_USB_1;
//     // char portName2[64] = MARVELMIND_USB_2;
//     // PLOGI << portName0;
//     // bool success0 = mmOpenPortByName(portName0);
//     // bool success1 = mmOpenPortByName(portName1);
//     // bool success2 = mmOpenPortByName(portName2);
//     // PLOGI << "S: " << success0 << success1 << success2;
//     // int success = static_cast<int>(success0) + static_cast<int>(success1) + static_cast<int>(success2);
//     // if(success == 2)
//     // {
//     //     PLOGI << "Successfully connected to 2 Marvelmind devices";
//     // }
//     // else if (success == 1)
//     // {
//     //     PLOGW << "Only able to connect to 1 Marvelmind device";
//     // }
//     // else if (success == 0)
//     // {
//     //     PLOGW << "Unable to connect to any Marvelmind devices";
//     // }
//     // else
//     // {
//     //     PLOGE << "Managed to connect to " << success << "Marvelmind devices. That shouldn't happen!";
//     // }

//     // Open loop
//     bool open = false;
//     int count = 0;
//     while(!open && count < 10000)
//     {
//         open = mmOpenPort();
//         count++;
//         usleep(1000);
//     }
//     PLOGI << "Open status: " << open;

//     // Get usb loop
//     bool usb_ready = false;
//     count = 0;
//     MarvelmindDeviceVersion usbDevVersion;
//     while(!usb_ready && count < 10000)
//     {
//         usb_ready = mmGetVersionAndId(MM_USB_DEVICE_ADDRESS, &usbDevVersion);
//         count++;
//         usleep(1000);
//     }
//     PLOGI <<"USB ready: " << usb_ready;
//     if(usb_ready)
//     {
//         PLOGI << "Device type: " << (usbDevVersion.fwVerDeviceType);
//         PLOGI << "Device is hedge: " << mmDeviceIsHedgehog(usbDevVersion.fwVerDeviceType);
//     }

//     // bool device_list_ok = false;
//     // count = 0;
//     // MarvelmindDevicesList device_list;
//     // while(!device_list_ok && count < 10)
//     // {
//     //     device_list_ok = mmGetDevicesList(&device_list);
//     //     count++;
//     //     PLOGI << count;
//     //     usleep(1000);
//     // }
//     // PLOGI << "Device list status: " << device_list_ok;

//     // if(!device_list_ok)
//     // {
//     //     PLOGW << "Could not retrieve list of Marvelmind devices";
//     // }
//     // else
//     // {
//     //     for(int i = 0; i < device_list.numDevices; i++)
//     //     {
//     //         MarvelmindDeviceInfo data = device_list.devices[i];
//     //         PLOGI << "Found device " << data.address;
//     //         // if(data.address == MARVELMIND_DEVICE_ID0 || data.address == MARVELMIND_DEVICE_ID1)
//     //         // {
//     //         //     PLOGI << "Found device " << data.address;
//     //             // if (data.isSleeping)
//     //             // {
//     //             //     PLOGI << "Device " << data.address << " is sleeping, triggering wake up";
//     //             //     mmWakeDevice(data.address);
//     //             // }
//     //             // else
//     //             // {
//     //             //     PLOGI << "Device " << data.address << " is already awake";
//     //             // }
//     //         // }
//     //     }
//     // }
// }

