#include "sampling.h"
#include "interpolation.h"

#include "ch.h"
#include "hal.h"

#include "wideband_config.h"

#include "port.h"
#include "io_pins.h"

#if 1 /* tunerstudio */
#include "livedata.h"
#endif

// Stored results
struct measure_results {
    float nernstAc;
    float nernstDc;
    float pumpCurrentSenseVoltage;
    float internalBatteryVoltage;
};

static struct measure_results results[AFR_CHANNELS];

static const struct inter_point lsu49_r_to_temp[] =
{
    {   80, 1030 },
    {  150,  890 },
    {  200,  840 },
    {  250,  805 },
    {  300,  780 },
    {  350,  760 },
    {  400,  745 },
    {  450,  730 },
    {  550,  705 },
    {  650,  685 },
    {  800,  665 },
    { 1000,  640 },
    { 1200,  630 },
    { 2500,  565 },
    // approximated by the greatest measurable sensor resistance
    { 5000,  500 }
};

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

constexpr float f_abs(float x)
{
    return x > 0 ? x : -x;
}

static THD_WORKING_AREA(waSamplingThread, 256);

static void SamplingThread(void*)
{
    int idx = 0;
    bool ready = false;
    float r_buf[AFR_CHANNELS][3];

    /* GD32: Insert 20us delay after ADC enable */
    chThdSleepMilliseconds(1);

    while(true)
    {
        auto result = AnalogSample();

        // Toggle the pin after sampling so that any switching noise occurs while we're doing our math instead of when sampling
        palTogglePad(NERNST_ESR_DRIVER_PORT, NERNST_ESR_DRIVER_PIN);

        for (int ch = 0; ch < AFR_CHANNELS; ch++) {
            struct measure_results *res = &results[ch];
            float *r = r_buf[ch];

            #ifdef BATTERY_INPUT_DIVIDER
                res->internalBatteryVoltage = result.ch[ch].BatteryVoltage;
            #endif

            r[idx] = result.ch[ch].NernstVoltage;

            if (ready)
            {
                // opposite_phase estimates where the previous sample would be had we not been toggling
                // AKA the absolute value of the difference between opposite_phase and r[idx] is the amplitude
                // of the AC component on the nernst voltage.  We have to pull this trick so as to use the past 3
                // samples to cancel out any slope in the DC (aka actual nernst cell output) from the AC measurement
                // See firmware/sampling.png for a drawing of what's going on here
                float opposite_phase = (r[idx] + r[(idx + 1) % 3]) / 2;

                // Compute AC (difference) and DC (average) components
                float nernstAcLocal = f_abs(opposite_phase - r[(idx + 2) % 3]);
                res->nernstDc = (opposite_phase + r[(idx + 1) % 3]) / 2;

                res->nernstAc =
                    (1 - ESR_SENSE_ALPHA) * res->nernstAc +
                    ESR_SENSE_ALPHA * nernstAcLocal;

                // Exponential moving average (aka first order lpf)
                res->pumpCurrentSenseVoltage =
                    (1 - PUMP_FILTER_ALPHA) * res->pumpCurrentSenseVoltage +
                    PUMP_FILTER_ALPHA * (result.ch[ch].PumpCurrentVoltage - result.VirtualGroundVoltageInt);
            }
        }

        idx++;
        if (idx == 3)
        {
            ready = true;
            idx = 0;
        }

        /* tunerstudio */
#if 1
        SamplingUpdateLiveData();
#endif
    }
}

void StartSampling()
{
    adcStart(&ADCD1, nullptr);
    chThdCreateStatic(waSamplingThread, sizeof(waSamplingThread), NORMALPRIO + 5, SamplingThread, nullptr);
}

float GetNernstAc(int ch)
{
    return results[ch].nernstAc;
}

float GetSensorInternalResistance(int ch)
{
    // Sensor is the lowside of a divider, top side is 22k, and 3.3v AC pk-pk is injected
    float totalEsr = ESR_SUPPLY_R / (VCC_VOLTS / GetNernstAc(ch) - 1);

    // There is a resistor between the opamp and Vm sensor pin.  Remove the effect of that
    // resistor so that the remainder is only the ESR of the sensor itself
    return totalEsr - VM_RESISTOR_VALUE;
}

float GetSensorTemperature(int ch)
{
    float esr = GetSensorInternalResistance(ch);

    if (esr > 5000)
    {
        return 0;
    }

    return interpolate_1d_float(lsu49_r_to_temp, ARRAY_SIZE(lsu49_r_to_temp), esr);
}

float GetNernstDc(int ch)
{
    return results[ch].nernstDc;
}

float GetPumpNominalCurrent(int ch)
{
    // Gain is 10x, then a 61.9 ohm resistor
    // Effective resistance with the gain is 619 ohms
    // 1000 is to convert to milliamperes
    constexpr float ratio = -1000 / (PUMP_CURRENT_SENSE_GAIN * LSU_SENSE_R);
    return results[ch].pumpCurrentSenseVoltage * ratio;
}

float GetInternalBatteryVoltage(int ch)
{
    return results[ch].internalBatteryVoltage;
}
