#include "MarvelmindWrapper.h"
#include "constants.h"
#include <marvelmind/marvelmind_api.h>

MarvelmindWrapper::MarvelmindWrapper()
{
    mmOpenPortByName(MARVELMIND_USB_1);
}