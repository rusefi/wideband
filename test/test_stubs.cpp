#include "status.h"
#include "can.h"

void SetStatus(int, wbo::Status)
{
}

wbo::Status GetCurrentStatus(int)
{
    return wbo::Status::NoHeat;
}

float GetRemoteBatteryVoltage()
{
    return 0;
}
