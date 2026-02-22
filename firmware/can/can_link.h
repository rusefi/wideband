#pragma once


#include <cstdint>

#include "hal.h"
#include "base_protocol_handler.h"
#include "port.h"

void ProcessLinkCanMessage(const CANRxFrame* msg, Configuration* configuration, struct CanStatusData* statusData);

extern const ProtocolHandler linkAfrTxHandler;
extern const ProtocolHandler linkEgtTxHandler;