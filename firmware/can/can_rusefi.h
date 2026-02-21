#pragma once

#include <cstdint>

#include "hal.h"
#include "base_protocol_handler.h"
#include "port.h"

void ProcessRusefiCanMessage(const CANRxFrame* msg, Configuration* configuration, struct CanStatusData* statusData);

extern CallbackHandler rusefiAfrTxHandler;