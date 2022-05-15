#pragma once

void StartSampling();

float GetNernstAc(int hi);
float GetSensorInternalResistance(int hi);
float GetSensorTemperature(int hi);
float GetNernstDc();

float GetPumpNominalCurrent();

float GetInternalBatteryVoltage();
