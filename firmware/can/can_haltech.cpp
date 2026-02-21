#include "can.h"
#include "hal.h"

#include "can_haltech.h"

#include "util/byteswap.h"
#include "port.h"
#include "status.h"
#include "can_helper.h"
#include "sampling.h"
#include "pump_dac.h"
#include "pwmout.h"

#include "heater_control.h"
#include "lambda_conversion.h"
#include "max3185x.h"
#include "../for_rusefi/wideband_can.h"

// Haltech protocol
// 1Mbps, big endian, DLC 8

#define HALTECH_WB2_BASE_ID         0x2B0

namespace haltech
{

enum class SensorFlags : uint8_t {
    None = 0,
    LowBattery = 1,
    HighBattery = 2,
    SensorShortCircuit = 3,
    SensorOpenCircuit = 4,
    SensorCold = 5,
};

// ID: WB2A 0x2B0
// ID: WB2B 0x2B4
// ID: WB2C 0x2B6
// ID: WB2D 0x2B8
struct AfrData1
{
    beuint16_t Lambda1;        // 32767 = Free Air, otherwise y = x/1024
    beuint16_t Lambda2;        // 0.001 Lambda/bit, 0.000 to 65.535 Lambda
    uint8_t RSense1;           // 0-255 Ohm; 1/1
    uint8_t RSense2;           // 0-255 Ohm; 1/1
    SensorFlags Sensor1Flags : 4;
    SensorFlags Sensor2Flags : 4;
    uint8_t VBatt;             // y = x * 20 / 255 Volts
} __attribute__((packed));

static_assert(sizeof(AfrData1) == 8);

} //namespace haltech


void SendHaltechAfrFormat(Configuration* configuration, uint8_t ch)
{
    // If channel 0 is configured for Haltech, channel 1 data is sent in the same message
    if (ch == 1 && configuration->afr[0].ExtraCanProtocol == CanAfrProtocol::Haltech)
        return;

    uint8_t channel0enabled = configuration->afr[0].ExtraCanProtocol == CanAfrProtocol::Haltech;

    #if AFR_CHANNELS > 1
    uint8_t channel1enabled = configuration->afr[1].ExtraCanProtocol == CanAfrProtocol::Haltech;
    #endif

    auto id = HALTECH_WB2_BASE_ID; // WB2A
    switch (configuration->afr[ch].ExtraCanIdOffset) {
        case 1:
            id += 4; break; // WB2B
        case 2:
            id += 6; break; // WB2C
        case 3:
            id += 8; break; // WB2D
    }

    CanTxTyped<haltech::AfrData1> frame(id, true);

    float vbatt = 0;

    if (channel0enabled) {
        const auto& sampler0 = GetSampler(0);
        float lambda1 = GetLambda(0);
        frame.get().Lambda1 = LambdaIsValid(0, lambda1) ? lambda1 * 1024 : 0;
        frame.get().RSense1 = sampler0.GetSensorInternalResistance();

        frame.get().Sensor1Flags = haltech::SensorFlags::None;

        vbatt = sampler0.GetInternalHeaterVoltage();
    }
    
    #if (AFR_CHANNELS > 1)

    if (channel1enabled) {
        const auto& sampler2 = GetSampler(1);
        float lambda2 = GetLambda(1);
        frame.get().Lambda2 = LambdaIsValid(1, lambda2) ? lambda2 * 1024 : 0;
        frame.get().RSense2 = sampler2.GetSensorInternalResistance();

        frame.get().Sensor2Flags = haltech::SensorFlags::None;
        float vbatt2 = sampler2.GetInternalHeaterVoltage();

        if (vbatt2 > vbatt) {
            vbatt = vbatt2;
        }
    }

    #endif
    
    frame.get().VBatt = vbatt * 255.0 / 20.0f;
}

#if (EGT_CHANNELS > 0)

#define HALTECH_TCA_BASE_ID         0x2CC

namespace haltech
{
    
struct EgtData
{
    beint16_t Egt[4];
} __attribute__((packed));

static_assert(sizeof(EgtData) == 8);

} //namespace haltech

// id 716 for "box a assigned to can tc 1-4" and
// id 717 for "box b assigned to can tc 5-8"
// Multiplier of 2381 divider of 5850 and offset of -250.

void SendHaltechEgtFormat(Configuration* configuration)
{
    auto id = HALTECH_TCA_BASE_ID + configuration->egt[0].ExtraCanIdOffset;

    CanTxTyped<haltech::EgtData> frame(id, true);

    for (uint8_t i = 0; i < EGT_CHANNELS; i++)
    {
        if (!configuration->egt[i].ExtraCanChannelEnabled)
            continue;

        frame.get().Egt[i] = (getEgtDrivers()[i].temperature + 250.0f) * 5850.0f / 2381.0f;
    }
}

#endif

#if ((AUX_INPUT_CHANNELS > 0) || (PWM_OUTPUT_CHANNELS > 0))

// IO_Expander protocol
// 50 Hz, big endian, DLC 8
#define HALTECH_IOEXPANDER_BASE_ID 0x2C0

namespace haltech
{

// AVI message: 4x 12-bit analog voltage inputs (stored as 16-bit big-endian)
// ID: Box A 0x2C0 (704), Box B 0x2C1 (705)
struct AviData
{
    beuint16_t AVI[4]; // Clamped to 4095 (0-5V scaled to 0-4095) in big-endian format
} __attribute__((packed));

static_assert(sizeof(AviData) == 8);

// DPI message: 2x digital pulse inputs
// Box A DPI1-2: 0x2C2 (706), Box A DPI3-4: 0x2C4 (708)
// Box B DPI1-2: 0x2C3 (707), Box B DPI3-4: 0x2C5 (709)
struct DpiChannel
{
    uint8_t duty;          // 0.1% per bit (0-250 represents 0-100%)
    uint8_t flags;         // bit 5 is state (on/off)
    uint8_t reserved;
    uint8_t period;        // period
} __attribute__((packed));

static_assert(sizeof(DpiChannel) == 4);

struct DpiData
{
    DpiChannel channels[2];
} __attribute__((packed));

static_assert(sizeof(DpiData) == 8);

// DPO control message: 2x digital pulse outputs
// Box A DPO1-2: 0x2D0 (720), DPO3-4: 0x2D2 (722)
// Box B DPO1-2: 0x2D1 (721), DPO3-4: 0x2D3 (723)
struct DpoChannel
{
    uint8_t duty;          // 0.4% per bit (0-250 = 0-100%)
    uint8_t flags;         // bit 4=safestate, bit 5=activestate (true=active LOW, false=active HIGH)
    uint8_t reserved;
    uint8_t period;        // 0.01ms per bit (8-bit: 0-2.55ms)
} __attribute__((packed));

static_assert(sizeof(DpoChannel) == 4);

struct DpoControl
{
    DpoChannel channels[2];
} __attribute__((packed));

static_assert(sizeof(DpoControl) == 8);

} //namespace haltech


void SendHaltechIO12Message(Configuration* configuration)
{
    // Determine Box A (704) or Box B (705) based on channel offset
    auto id = HALTECH_IOEXPANDER_BASE_ID + configuration->afr[0].ExtraCanIdOffset;
    
    CanTxTyped<haltech::AviData> frame(id, true);

    for (uint8_t i = 0; i < AUX_INPUT_CHANNELS && i < 4; i++)
    {
        if (configuration->ioExpanderConfig.IOInputsEnabled & (1 << i))
        {
            float voltage = GetAuxInputVoltage(i);
            uint16_t raw = (uint16_t)(voltage * 4095.0f / 5.0f) & 0xFFF; // Scale 0-5V to 0-4095
            frame.get().AVI[i] = raw;
        }
        else
        {
            frame.get().AVI[i] = 0; // Input disabled, report as 0V
        }
    }

    // DPI messages would go here if we had digital pulse input hardware
    // For now, this is left as a stub for future implementation
    // When implemented, would send:
    // - Message 1 (706/707): DPI1 and DPI2
    // - Message 2 (708/709): DPI3 and DPI4
    // with duty cycle (0-250 for 0-100%), state (bit 5), and period
}

static void ProcessDpoChannel(const haltech::DpoChannel* channel, uint8_t channelNumber)
{
    uint8_t duty_raw = channel->duty;                    // 0-250 (0.4% per bit)
    bool activestate = (channel->flags >> 5) & 0x01;     // Bit 5: true = active LOW, false = active HIGH
    // bool safestate = (channel->flags >> 4) & 0x01;       // Bit 4: safe state level
    // uint8_t period = channel->period;                    // 0.01ms per bit (8-bit: 0-2.55ms)

    // Apply output control based on duty cycle
    float pwm_duty = duty_raw / 250.0f;  // 0.0 to 1.0
    if (pwm_duty > 1.0f) {
        pwm_duty = 1.0f;
    }

    // If active LOW mode, invert the duty cycle
    if (activestate) {
        pwm_duty = 1.0f - pwm_duty;
    }

    SetAuxPwmDuty(channelNumber, pwm_duty);

    // TODO: Apply frequency to PWM timer if supported by hardware.

    // TODO: Implement CAN timeout monitoring (2000ms timeout)
    // On CAN timeout or safe mode trigger, set pin to safe state:
    // Store safestate to permanent config and have a periodic check that sets the output to safestate
    // if no message received within 2000ms
    // SetAuxPwmDuty(channelNumber, safestate ? 1.0f : 0.0f);
}

void ProcessHaltechIO12Message(const CANRxFrame* frame, Configuration* configuration)
{
    (void) configuration;

    auto id = CAN_ID(*frame);
    
    const uint16_t DPO12_BASE_ID = 0x2D0;  // 720 for Box A, 721 for Box B
    const uint16_t DPO34_BASE_ID = 0x2D2;  // 722 for Box A, 723 for Box B

    if (id < DPO12_BASE_ID || id > DPO34_BASE_ID + 1)
    {
        return; // Not a DPO control message
    }

    const haltech::DpoControl* dpo = reinterpret_cast<const haltech::DpoControl*>(frame->data8);

    uint8_t baseChannel;
    switch (id)
    {
        case DPO12_BASE_ID:      // 0x2D0 - DPO1-2 Box A
        case DPO12_BASE_ID + 1:  // 0x2D1 - DPO1-2 Box B
            baseChannel = 0;
            break;
        case DPO34_BASE_ID:      // 0x2D2 - DPO3-4 Box A
        case DPO34_BASE_ID + 1:  // 0x2D3 - DPO3-4 Box B
            baseChannel = 2;
            break;
    }

    // Process both channels in this message
    if (configuration->ioExpanderConfig.IOOutputsEnabled & (1 << baseChannel)) {
        ProcessDpoChannel(&dpo->channels[0], baseChannel);
    }
    
    if (configuration->ioExpanderConfig.IOOutputsEnabled & (1 << (baseChannel + 1))) {
        ProcessDpoChannel(&dpo->channels[1], baseChannel + 1);
    }
}

#else

void SendHaltechIO12Message(Configuration* configuration)
{
    (void) configuration;
    // No aux inputs, so nothing to send
}

void ProcessHaltechIO12Message(const CANRxFrame* frame, Configuration* configuration)
{
    (void) frame;
    (void) configuration;
    // No aux inputs, so nothing to process
}

#endif
