#include "ch.h"
#include "hal.h"

#include "io_pins.h"
#include "livedata.h"

#include "sampling.h"
#include "port.h"

static Sampler samplers[AFR_CHANNELS];

const ISampler& GetSampler(int ch)
{
    return samplers[ch];
}

static THD_WORKING_AREA(waSamplingThread, 256);

#ifdef BOARD_HAS_VOLTAGE_SENSE
static float supplyVoltage = 0;

float GetSupplyVoltage()
{
    return supplyVoltage;
}
#endif

#if AUX_INPUT_CHANNELS > 0

static float auxInputVoltage[AUX_INPUT_CHANNELS] = {};
float GetAuxInputVoltage(uint8_t idx)
{
    if (idx >= AUX_INPUT_CHANNELS) {
        return 0.0f;
    }

    return auxInputVoltage[idx];
}

#endif

static float mcuTemp = 0;
float GetMcuTemperature()
{
    return mcuTemp;
}

static void SamplingThread(void*)
{
    chRegSetThreadName("Sampling");

    SetupESRDriver(GetSensorType());

    /* GD32: Insert 20us delay after ADC enable */
    chThdSleepMilliseconds(1);

    AnalogSampleStart();

    while(true)
    {
        auto result = AnalogSampleFinish();

        // Toggle the pin after sampling so that any switching noise occurs while we're doing our math instead of when sampling
        ToggleESRDriver(GetSensorType());

        AnalogSampleStart();

        #ifdef BOARD_HAS_VOLTAGE_SENSE
        supplyVoltage = result.SupplyVoltage;
        #endif
        mcuTemp = result.McuTemp;

        for (int ch = 0; ch < AFR_CHANNELS; ch++)
        {
            samplers[ch].ApplySample(result.ch[ch], result.VirtualGroundVoltageInt);
        }

        #if AUX_INPUT_CHANNELS > 0
        for (int i = 0; i < AUX_INPUT_CHANNELS; i++)
        {
            auxInputVoltage[i] = result.AuxInputVoltage[i] * AUX_INPUT_GAIN;
        }
        #endif

#if defined(TS_ENABLED)
        /* tunerstudio */
        SamplingUpdateLiveData();
#endif
    }
}

void StartSampling()
{
    for (int i = 0; i < AFR_CHANNELS; i++)
    {
        samplers[i].Init();
    }

    PortPrepareAnalogSampling();

    chThdCreateStatic(waSamplingThread, sizeof(waSamplingThread), NORMALPRIO + 5, SamplingThread, nullptr);
}
