#pragma once

#include <cstdint>

#include "hal.h"
#include "port.h"

void SendMsIoBoxFormat(Configuration* cfg);
void HandleMsIoBoxCanMessage(const CANRxFrame* msg, Configuration* configuration, struct CanStatusData* statusData);