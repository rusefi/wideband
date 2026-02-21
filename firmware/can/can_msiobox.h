#pragma once

#include <cstdint>

#include "hal.h"
#include "port.h"
#include "base_protocol_handler.h"

void ProcessMsIoBoxCanMessage(const CANRxFrame* msg, Configuration* configuration);
extern IoHandler msIoBoxTxHandler;