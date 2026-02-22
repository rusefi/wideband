#include "can.h"
#include "hal.h"

#include "util/byteswap.h"

#include "can_helper.h"
#include "can_msiobox.h"

#include "port.h"
#include "status.h"
#include "heater_control.h"
#include "lambda_conversion.h"
#include "sampling.h"
#include "pump_dac.h"
#include "max3185x.h"
#include "auxout.h"
#include "pwmout.h"

#if (IO_EXPANDER_ENABLED > 0)

// Default period
#define MS_IOBOX_PERIOD_MS         20

static ProtocolHandler msIoBoxTxHandler = MakeProtocolHandler<&SendMsIoBoxFormat>(MS_IOBOX_PERIOD_MS);

// MS IoBox protocol

#define MS_IOBOX_BASE_ID         0x200
#define MS_IOBOX_OFFSSET         0x20
#define MS_IOBOX_DEVICE_ID(n)    (MS_IOBOX_BASE_ID + (MS_IOBOX_OFFSSET)*(n))

#define MS_IOBOX_PING          0x00
#define MS_IOBOX_CONFIG        0x01
#define MS_IOBOX_SET_PWM(n)    (0x02 + ((n) & 0x03))
#define MS_IOBOX_LAST_IN       0x05

/* Packets from device to MS3 */
#define MS_IOBOX_WHOAMI        0x08
#define MS_IOBOX_ADC14         0x09
#define MS_IOBOX_ADC57         0x0A

namespace msiobox
{

struct pwm_settings {
    beuint16_t on;
    beuint16_t off;
} __attribute__ ((packed));

/* Base + 0x00 */
/* "Are you there?" packet with zero payload */

/* Base + 0x01 */
struct cfg {
    uint8_t pwm_mask;   // 0 - On/Off, 1 - PWM
    uint8_t pad0;
    uint8_t tachin_mask;
    uint8_t pad1;
    uint8_t adc_broadcast_interval;     // mS
    uint8_t tach_broadcast_interval;    // mS
    uint8_t pad2[2];
} __attribute__((packed));

/* Base + 0x02, 0x03, 0x04 */
struct pwm {
    pwm_settings ch[2];
} __attribute__ ((packed));

static_assert(sizeof(pwm) == 8);

/* Base + 0x05 */
struct pwm_last {
    pwm_settings ch[1];
    uint8_t out_state;
} __attribute__ ((packed));

static_assert(sizeof(pwm_last) == 5);

/* Base + 0x08 */
struct whoami {
    uint8_t version;
    uint8_t pad[3];
    beuint16_t pwm_period;      // PWM clock periods in 0.01 uS
    beuint16_t tachin_period;   // Tach-in clock periods in 0.01 uS
} __attribute__((packed));

static_assert(sizeof(whoami) == 8);

/* Base + 0x09 */
struct adc14 {
    beuint16_t adc[4];
} __attribute__((packed));

static_assert(sizeof(adc14) == 8);

/* Base + 0x0A */
struct adc57 {
    uint8_t inputs;
    uint8_t pad;
    beuint16_t adc[3];
} __attribute__((packed));

static_assert(sizeof(adc57) == 8);

} //namespace msiobox

static bool configured = false;
static uint8_t pwm_mask = 0x00;
static uint8_t tachin_mask = 0x00;
static uint8_t tach_broadcast_interval = 20;

/*
 * TODO: validate
 * Scale value to 0 .. 1023
 * MegaSquirt IOBox has 0..5V ADC input range
 */
static uint16_t CanIoBoxGetAdc(size_t index)
{
    /* Total 7 ADC inputs, mapping:
     * 1 - AUX left
     * 2 - AUX right
     * 3 - AUX out left voltage
     * 4 - AUX out right voltage
     * 5 - WBO supply voltage
     * 6 - Left sensor heater supply
     * 7 - Right sensor heater supply */
    switch (index) {
        case 1: // AUX Left
        case 2: // AUX Right
        {
        #if (AUX_INPUT_CHANNELS > 0)
            if (index > AUX_INPUT_CHANNELS) {
                return 0;
            }
            float voltage = GetAuxInputVoltage(index - 1);
            if (voltage < 0) voltage = 0;
            if (voltage > 5) voltage = 5;
            return (uint16_t)(voltage / 5.0 * 1024.0);
        #else
            return 0;
        #endif
        }
        case 3: // AUX out left voltage
        case 4: // AUX out right voltage
            return 0;
        case 5: // WBO supply voltage
            return 0;
        case 6:
        case 7:
        {
            const auto& sampler = GetSampler(index - 6);
            /* TODO: clamp */
            return sampler.GetInternalHeaterVoltage() / 25.5 * 1024;
        }
        default:
            return 0.0;
    }
}

void SendMsIoBoxFormat(Configuration* configuration)
{
    // if (!configuration->ioExpanderConfig.enable_tx)
    //    return;

    if (!configured)
        return;

    const uint32_t base = MS_IOBOX_DEVICE_ID(configuration->ioExpanderConfig.Offset);

    if(1) // Create a scope to ensure frame is sent early
    {
        CanTxTyped<msiobox::adc14> frame1(base + MS_IOBOX_ADC14, false);

        for (size_t i = 0; i < 4; i++) {
            frame1->adc[i] = CanIoBoxGetAdc(i);
        }
    }

    if (1) // Create a scope to ensure frame is sent early
    {
        CanTxTyped<msiobox::adc57> frame2(base + MS_IOBOX_ADC57, false);

        for (size_t i = 0; i < 3; i++) {
            frame2.get().adc[i] = CanIoBoxGetAdc(4 + i);
        }
    }
}

void ProcessMsIoBoxCanMessage(const CANRxFrame* fr, Configuration* cfg)
{
    if (!(cfg->ioExpanderConfig.Protocol == CanIoProtocol::MsIoBox)) {
        return;
    }
    // For now only support standard IDs
    if (fr->IDE != CAN_IDE_STD) {
        return;
    }

    const uint32_t base = MS_IOBOX_DEVICE_ID(cfg->ioExpanderConfig.Offset);
    uint32_t frame_id = fr->SID;

    if ((frame_id < base) || (frame_id > base + MS_IOBOX_LAST_IN))
        return;

    if ((frame_id == base + MS_IOBOX_PING) && (fr->DLC == 0))
    {
        CanTxTyped<msiobox::whoami> frame(base + MS_IOBOX_WHOAMI, false);

        frame->version = 1;

        // PWM clock periods in 0.01 uS, equal to clock freq / 100 * 1000
        frame->pwm_period = 5000;
        // Tach-in clock periods in 0.01 uS, equal to clock freq / 100 * 1000
        frame->tachin_period = 66;

        return;
    }
    
    if ((frame_id == base + MS_IOBOX_CONFIG) && (fr->DLC == sizeof(msiobox::cfg)))
    {
        const msiobox::cfg *iobox_config = reinterpret_cast<const msiobox::cfg *>(fr->data8);

        //can0       201   [8]  00 00 00 00 14 14 00 00
        pwm_mask = iobox_config->pwm_mask;
        tachin_mask = iobox_config->tachin_mask;
        tach_broadcast_interval = iobox_config->tach_broadcast_interval;

        msIoBoxTxHandler.interval_ms = iobox_config->adc_broadcast_interval;

        configured = true;

        return;
    }

    if ((frame_id >= base + MS_IOBOX_SET_PWM(0)) &&
        (frame_id <= base + MS_IOBOX_SET_PWM(2)) &&
        (fr->DLC == sizeof(msiobox::pwm)))
    {
        const msiobox::pwm *pwm = reinterpret_cast<const msiobox::pwm *>(fr->data8);

        /* Two first channels are mapped to analog outputs */
        if (frame_id == base + MS_IOBOX_SET_PWM(0)) {
            for (size_t n = 0; n < AFR_CHANNELS; n++) {
                // if allowed to control DAC output over CAN
                if (cfg->auxOutputSource[n] != AuxOutputMode::IOExpander) {
                    continue;
                }

                uint32_t period = pwm->ch[n].off + pwm->ch[n].on;
                float duty = (period == 0) ? 0 : (float)pwm->ch[n].on / period;
                SetAuxDac(n, 5.0 * duty);
            }
        }
        #if (PWM_OUTPUT_CHANNELS > 0)
        /* Second set of PWM channels */
        else if (frame_id == base + MS_IOBOX_SET_PWM(1)) {
            for (size_t n = 0; n < PWM_OUTPUT_CHANNELS; n++) {
                uint32_t period = pwm->ch[n].off + pwm->ch[n].on;
                float duty = (period == 0) ? 0 : (float)pwm->ch[n].on / period;
                SetAuxPwmDuty(n, duty);
            }
        }
        #endif

        /* TODO: PWM periods */
        return;
    }

    if ((frame_id == base + MS_IOBOX_SET_PWM(3)) && (fr->DLC == sizeof(msiobox::pwm_last)))
    {
        const msiobox::pwm_last *pwm = reinterpret_cast<const msiobox::pwm_last *>(fr->data8);
        
        /* TODO: PWM periods and outputs */
        (void)pwm;

        return;
    }
}

#endif