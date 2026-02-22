#pragma once


#include <cstdint>

#include "hal.h"
#include "base_protocol_handler.h"
#include "port.h"

void ProcessHaltechIO12Message(const CANRxFrame* frame, Configuration* configuration);

extern const ProtocolHandler haltechAfrTxHandler;
extern const ProtocolHandler haltechEgtTxHandler;
extern const ProtocolHandler haltechIoTxHandler;

