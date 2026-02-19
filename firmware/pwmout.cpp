#include "ch.h"
#include "hal.h"

#include "pwm.h"
#include "port.h"
#include "io_pins.h"

#include "wideband_config.h"

#include <rusefi/math.h>

#ifdef AUXOUT_OD_PWM_DEVICE

#ifndef AUXOUT_OD_PWM_OUTPUT_MODE
// Use complementary outputs (active low for open drain behavior)
#define AUXOUT_OD_PWM_OUTPUT_MODE PWM_OUTPUT_ACTIVE_LOW | PWM_COMPLEMENTARY_OUTPUT_ACTIVE_LOW
#endif

// PWM configuration for TIM1 (shared by both channels)
// Both channels will have the same frequency but independent duty cycles
// Default: 1 kHz
static PWMConfig auxPwmConfig = {
    .frequency = STM32_SYSCLK,
    .period = 48000, // ~1 kHz at 48 MHz
    .callback = nullptr,
    .channels = {
        [0] = {.mode = PWM_OUTPUT_DISABLED, .callback = nullptr},
#if (PWM_OUTPUT_CHANNELS > 0)
        [AUXOUT_OD_PWM_CHANNEL_L] = {.mode = AUXOUT_OD_PWM_OUTPUT_MODE, .callback = nullptr},
#endif
#if (PWM_OUTPUT_CHANNELS > 1)
        [AUXOUT_OD_PWM_CHANNEL_R] = {.mode = AUXOUT_OD_PWM_OUTPUT_MODE, .callback = nullptr},
#endif
        [3] = {.mode = PWM_OUTPUT_DISABLED, .callback = nullptr}
    },
    .cr2 = 0,
#if STM32_PWM_USE_ADVANCED
    .bdtr = 0,
#endif
    .dier = 0
};

static Pwm auxPwm(AUXOUT_OD_PWM_DEVICE);
static float currentFreq = 1000.0f;

static const uint8_t channelMap[PWM_OUTPUT_CHANNELS] = {
    AUXOUT_OD_PWM_CHANNEL_L,
#if (PWM_OUTPUT_CHANNELS > 1)
    AUXOUT_OD_PWM_CHANNEL_R,
#endif
};

void SetAuxPwmDuty(uint8_t channel, float duty)
{
    if (channel >= PWM_OUTPUT_CHANNELS) {
        return;
    }

    // Clamp duty cycle to valid range [0, 1]
    duty = clampF(0, duty, 1);
    
    auxPwm.SetDuty(channelMap[channel], duty);
}

void SetAuxPwmFreq(float frequency)
{
    // Use default frequency if not specified or invalid
    if (frequency <= 0) {
        frequency = 1000.0f;
    }

    // Only update if frequency changed
    if (frequency != currentFreq) {
        uint32_t newPeriod = (uint32_t)(STM32_SYSCLK / frequency);
        auxPwm.Stop();
        auxPwmConfig.period = newPeriod;
        auxPwm.Start(auxPwmConfig);
        currentFreq = frequency;
    }
}

void InitAuxPwm()
{
    auxPwm.Start(auxPwmConfig);
    
    // Initialize all channels to 0% duty
    for (uint8_t ch = 0; ch < PWM_OUTPUT_CHANNELS; ch++) {
        auxPwm.SetDuty(channelMap[ch], 0.0f);
    }
}

#else /* AUXOUT_OD_PWM_DEVICE */

void InitAuxPwm()
{
}

void SetAuxPwmDuty(uint8_t channel, float duty)
{
    (void)channel;
    (void)duty;
}

void SetAuxPwmFreq(float frequency)
{
    (void)frequency;
}

#endif
