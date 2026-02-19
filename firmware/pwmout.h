#pragma once

#include <cstdint>

void InitAuxPwm();

// Set PWM duty cycle for a specific channel (0.0 to 1.0)
void SetAuxPwmDuty(uint8_t channel, float duty);

// Set PWM frequency for all channels (Hz)
// NOTE: On boards where channels share the same timer,
// this affects ALL channels simultaneously.
void SetAuxPwmFreq(float frequency);
