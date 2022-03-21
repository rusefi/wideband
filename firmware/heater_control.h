#pragma once

#include <cstdint>

enum class HeaterState
{
    Preheat,
    WarmupRamp,
    ClosedLoop,
    Stopped,
};

void StartHeaterControl();
bool IsRunningClosedLoop();
void SetBatteryVoltage(float vbatt);
void SetHeaterAllowed(bool allowed);
float GetHeaterDuty();
HeaterState GetHeaterState();
const char* describeHeaterState(HeaterState state);
