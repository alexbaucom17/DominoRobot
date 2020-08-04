#include "MarvelmindWrapper.h"
#include "constants.h"

extern "C" {
    #include <marvelmind/marvelmind_api.h>
}

MarvelmindWrapper::MarvelmindWrapper()
{
    marvelmindAPILoad();
    mmOpenPortByName(MARVELMIND_USB_1);
}

MarvelmindWrapper::~MarvelmindWrapper()
{
    marvelmindAPIFree();
}