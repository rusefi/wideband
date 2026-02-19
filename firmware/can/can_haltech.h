#pragma once

#include <cstdint>

#include "hal.h"
#include "port.h"

void SendHaltechAfrFormat(Configuration* configuration, uint8_t ch);
void SendHaltechEgtFormat(Configuration* configuration);

void SendHaltechIO12Message(Configuration* configuration);
void ProcessHaltechIO12Message(const CANRxFrame* frame, Configuration* configuration);

