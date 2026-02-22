#pragma once


#include <cstdint>

#include "hal.h"
#include "base_protocol_handler.h"
#include "port.h"


extern const ProtocolHandler ecuMasterAfrTxHandler;
extern const ProtocolHandler ecuMasterClassicEgtTxHandler;
extern const ProtocolHandler ecuMasterBlackEgtTxHandler;
extern const ProtocolHandler ecuMasterSwitchBoardTxHandler;

void HandleEcuMasterCanMessage(const CANRxFrame* msg, Configuration* configuration);