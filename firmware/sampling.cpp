#include "sampling.h"
#include "interpolation.h"

#include "ch.h"
#include "hal.h"

#include "wideband_config.h"

#include "port.h"
#include "io_pins.h"

// Stored results
static float nernstAcHi = 0;
static float nernstAcLo = 0;
static float nernstDc = 0;
static float pumpCurrentSenseVoltage = 0;
static float internalBatteryVoltage = 0;

enum {
    STATE_Z1 = 0,
    STATE_PULLUP = 1,
    STATE_Z2 = 2,
    STATE_PULLDOWN = 3
};

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

/* Get previous sample index from current index and offset, offset in -7..0 range */
static unsigned int cb_idx(unsigned int idx, int offset)
{
    //return (idx + 8 + offset) % 8;
    return (idx + 8 + offset) & 0x07;
}

static void SamplingThread(void*)
{
    unsigned int idx = 0;
    int state = STATE_Z1;
    bool ready = false;
    /* store 2 full cycles */
    float r[8];

    /* GD32: Insert 20us delay after ADC enable */
    chThdSleepMilliseconds(1);

    while(true)
    {
        /* TODO: run for all channels */
        int ch = 0;

        auto result = AnalogSample();

        int next_state = (state + 1) % 4;
        // Toggle the pin after sampling so that any switching noise occurs while we're doing our math instead of when sampling
        if ((next_state == STATE_Z1) || (next_state == STATE_Z2)) {
            /* set pin to hi-z (input) mode */
            palSetPadMode(NERNST_ESR_DRIVER_PORT, NERNST_ESR_DRIVER_PIN, PAL_MODE_INPUT);
        } else if (next_state == STATE_PULLUP) {
            /* first set level then switch to output mode to avoid nidle to oposite level */
            palSetPad(NERNST_ESR_DRIVER_PORT, NERNST_ESR_DRIVER_PIN);
            palSetPadMode(NERNST_ESR_DRIVER_PORT, NERNST_ESR_DRIVER_PIN, PAL_MODE_OUTPUT_PUSHPULL);
        } else /* if (next_state == STATE_PULLDOWN) */ {
            /* first set level then switch to output mode to avoid nidle to oposite level */
            palClearPad(NERNST_ESR_DRIVER_PORT, NERNST_ESR_DRIVER_PIN);
            palSetPadMode(NERNST_ESR_DRIVER_PORT, NERNST_ESR_DRIVER_PIN, PAL_MODE_OUTPUT_PUSHPULL);
        }

        #ifdef BATTERY_INPUT_DIVIDER
            internalBatteryVoltage = result.ch[ch].BatteryVoltage;
        #endif

        r[idx] = result.ch[ch].NernstVoltage;

        if (ready)
        {
#if 0
            // opposite_phase estimates where the previous sample would be had we not been toggling
            // AKA the absolute value of the difference between opposite_phase and r[idx] is the amplitude
            // of the AC component on the nernst voltage.  We have to pull this trick so as to use the past 3
            // samples to cancel out any slope in the DC (aka actual nernst cell output) from the AC measurement
            // See firmware/sampling.png for a drawing of what's going on here
            float opposite_phase = (r[idx] + r[(idx + 1) % 3]) / 2;

            // Compute AC (difference) and DC (average) components
            float nernstAcLocal = f_abs(opposite_phase - r[(idx + 2) % 3]);
            nernstDc = (opposite_phase + r[(idx + 1) % 3]) / 2;

            nernstAc =
                (1 - ESR_SENSE_ALPHA) * nernstAc +
                ESR_SENSE_ALPHA * nernstAcLocal;
#else
            if ((state == STATE_PULLUP) || (state == STATE_PULLDOWN)) {
                /* get 'not affected' nernst voltage (two closest states when we were not driving ESR circuit) */
                nernstDc = (r[cb_idx(idx, -1)] + r[cb_idx(idx, -3)]) / 2;
                float pullPhase = (r[cb_idx(idx, 0)] + r[cb_idx(idx, -4)]) / 2;

                float nernstAcLocal = f_abs(nernstDc - pullPhase) * 2;
                if (state == STATE_PULLUP) {
                    nernstAcHi =
                        (1 - ESR_SENSE_ALPHA) * nernstAcHi +
                        ESR_SENSE_ALPHA * nernstAcLocal;
                } else {
                    nernstAcLo =
                        (1 - ESR_SENSE_ALPHA) * nernstAcLo +
                        ESR_SENSE_ALPHA * nernstAcLocal;
                }
            } else {
                /* get 'not affected' nernst voltage (two closest states when we were not driving ESR circuit) */
                nernstDc = (r[cb_idx(idx, 0)] + r[cb_idx(idx, -2)]) / 2;
            }
#endif

            // Exponential moving average (aka first order lpf)
            pumpCurrentSenseVoltage =
                (1 - PUMP_FILTER_ALPHA) * pumpCurrentSenseVoltage +
                PUMP_FILTER_ALPHA * (result.ch[ch].PumpCurrentVoltage - result.VirtualGroundVoltageInt);
        }

        idx++;
        if (idx == 8)
        {
            ready = true;
            idx = 0;
        }
        state = next_state;

    }
}

void StartSampling()
{
    adcStart(&ADCD1, nullptr);
    chThdCreateStatic(waSamplingThread, sizeof(waSamplingThread), NORMALPRIO + 5, SamplingThread, nullptr);
}

float GetNernstAc(int hi)
{
    return hi ? nernstAcHi : nernstAcLo;
}

float GetSensorInternalResistance(int hi)
{
    // Sensor is the lowside of a divider, top side is 22k, and 3.3v AC pk-pk is injected
    float totalEsr = ESR_SUPPLY_R / (VCC_VOLTS / GetNernstAc(hi) - 1);

    // There is a resistor between the opamp and Vm sensor pin.  Remove the effect of that
    // resistor so that the remainder is only the ESR of the sensor itself
    return totalEsr - VM_RESISTOR_VALUE;
}

float GetSensorTemperature(int hi)
{
    float esr = GetSensorInternalResistance(hi);

    if (esr > 5000)
    {
        return 0;
    }

    return interpolate_1d_float(lsu49_r_to_temp, ARRAY_SIZE(lsu49_r_to_temp), esr);
}

float GetNernstDc()
{
    return nernstDc;
}

float GetPumpNominalCurrent()
{
    // Gain is 10x, then a 61.9 ohm resistor
    // Effective resistance with the gain is 619 ohms
    // 1000 is to convert to milliamperes
    constexpr float ratio = -1000 / (PUMP_CURRENT_SENSE_GAIN * LSU_SENSE_R);
    return pumpCurrentSenseVoltage * ratio;
}

float GetInternalBatteryVoltage()
{
    return internalBatteryVoltage;
}
