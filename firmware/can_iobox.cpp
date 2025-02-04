#include "can.h"
#include "hal.h"

#include "can_helper.h"
#include "can_iobox.h"

#include "port.h"
#include "auxout.h"
#include "sampling.h"
#include "byteswap.h"

#define CAN_IOBOX_BASE0         0x200
#define CAN_IOBOX_BASE1         0x220
#define CAN_IOBOX_BASE2         0x240

/* Packets from MS3 to device */
#define CAN_IOBOX_PING          0x00
#define CAN_IOBOX_CONFIG        0x01
#define CAN_IOBOX_SET_PWM(n)    (0x02 + ((n) & 0x03))
#define CAN_IOBOX_LAST_IN       0x05

/* Packets from device to MS3 */
#define CAN_IOBOX_WHOAMI        0x08
#define CAN_IOBOX_ADC14         0x09
#define CAN_IOBOX_ADC57         0x0A

struct pwm_settings {
    beuint16_t on;
    beuint16_t off;
} __attribute__ ((packed));

/* Base + 0x00 */
/* "Are you there?" packet with zero payload */

/* Base + 0x01 */
struct iobox_cfg {
    uint8_t pwm_mask;   // 0 - On/Off, 1 - PWM
    uint8_t pad0;
    uint8_t tachin_mask;
    uint8_t pad1;
    uint8_t adc_broadcast_interval;     // mS
    uint8_t tach_broadcast_interval;    // mS
    uint8_t pad2[2];
} __attribute__((packed));

/* Base + 0x02, 0x03, 0x04 */
struct iobox_pwm {
    pwm_settings ch[2];
} __attribute__ ((packed));

static_assert(sizeof(iobox_pwm) == 8);

/* Base + 0x05 */
struct iobox_pwm_last {
    pwm_settings ch[1];
    uint8_t out_state;
} __attribute__ ((packed));

static_assert(sizeof(iobox_pwm_last) == 5);

/* Base + 0x08 */
struct iobox_whoami {
    uint8_t version;
    uint8_t pad[3];
    beuint16_t pwm_period;      // PWM clock periods in 0.01 uS
    beuint16_t tachin_period;   // Tach-in clock periods in 0.01 uS
} __attribute__((packed));

static_assert(sizeof(iobox_whoami) == 8);

/* Base + 0x09 */
struct iobox_adc14 {
    beuint16_t adc[4];
} __attribute__((packed));

static_assert(sizeof(iobox_adc14) == 8);

/* Base + 0x0A */
struct iobox_adc57 {
    uint8_t inputs;
    uint8_t pad;
    beuint16_t adc[3];
} __attribute__((packed));

static_assert(sizeof(iobox_adc57) == 8);

static bool configured = false;

static uint8_t pwm_mask = 0x00;
static uint8_t tachin_mask = 0x00;
static uint8_t adc_broadcast_interval = 20;
static uint8_t tach_broadcast_interval = 20;

static uint32_t CanIoBoxBaseId()
{
    const auto cfg = GetConfiguration();

    // three standart IDs
    if (cfg->iobox.idx <= 2) {
        return (CAN_IOBOX_BASE0 + cfg->iobox.idx * 0x20);
    }

    return ((cfg->iobox.IDE == CAN_IDE_STD) ? cfg->iobox.SID : cfg->iobox.EID);
}

static uint8_t CanIoBoxIde()
{
    const auto cfg = GetConfiguration();

    return cfg->iobox.IDE;
}

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
    case 0 ... 4:
        return 0;
    break;
    case 5:
        /* TODO: */
        return 0;
    case 6:
    case 7:
        {
            const auto& sampler = GetSampler(index - 6);

            /* TODO: clamp */
            return sampler.GetInternalHeaterVoltage() / 25.5 * 1024;
        }
    break;

    default:
    return 0.0;
    }
}

int CanIoBoxRx(const CANRxFrame *fr)
{
    const auto cfg = GetConfiguration();

    if (!cfg->iobox.enable_rx)
        return 0;

    if (CanIoBoxIde() != fr->IDE)
        return 0;

    const uint32_t base = CanIoBoxBaseId();
    uint32_t id;
    if (fr->IDE == CAN_IDE_STD) {
        id = fr->SID;
    } else {
        id = fr->EID;
    }

    if ((id < base) ||
        (id > base + CAN_IOBOX_LAST_IN))
        return 0;

    if ((id == base + CAN_IOBOX_PING) && (fr->DLC == 0))
    {
        CanTxTyped<iobox_whoami> frame(base + CAN_IOBOX_WHOAMI, false);

        frame.get().version = 1;
        // PWM clock periods in 0.01 uS, equal to clock freq / 100 * 1000
        frame.get().pwm_period = 5000;
        // Tach-in clock periods in 0.01 uS, equal to clock freq / 100 * 1000
        frame.get().tachin_period = 66;

        return 0;
    }
    else if ((id == base + CAN_IOBOX_CONFIG) && (fr->DLC == sizeof(iobox_cfg)))
    {
        const iobox_cfg *cfg = reinterpret_cast<const iobox_cfg *>(fr->data8);

        //can0       201   [8]  00 00 00 00 14 14 00 00
        pwm_mask = cfg->pwm_mask;
        tachin_mask = cfg->tachin_mask;
        adc_broadcast_interval = cfg->adc_broadcast_interval;
        tach_broadcast_interval = cfg->tach_broadcast_interval;

        configured = true;

        return 0;
    }
    else if ((id >= base + CAN_IOBOX_SET_PWM(0)) &&
             (id <= base + CAN_IOBOX_SET_PWM(2)) &&
             (fr->DLC == sizeof(iobox_pwm)))
    {
        const iobox_pwm *pwm = reinterpret_cast<const iobox_pwm *>(fr->data8);

        /* Two first channels are mapped to analog outputs */
        if (fr->EID == base + CAN_IOBOX_SET_PWM(0)) {
            for (size_t n = 0; n < 2; n++) {
                // if allowed to control DAC output over CAN
                if (cfg->auxOutputSource[n] != AuxOutputMode::MsIoBox) {
                    continue;
                }

                uint32_t period = pwm->ch[n].off + pwm->ch[n].on;
                if (period == 0) {
                    SetAuxDac(n, 0);
                } else {
                    SetAuxDac(n, 5.0 * pwm->ch[n].on / period);
                }
            }
        }

        /* TODO: PWM periods */
        return 0;
    }
    else if ((id == base + CAN_IOBOX_SET_PWM(3)) &&
             (fr->DLC == sizeof(iobox_pwm_last)))
    {
        const iobox_pwm_last *pwm = reinterpret_cast<const iobox_pwm_last *>(fr->data8);

        /* TODO: PWM periods and outputs */
        return 0;
    }

    /* Should not happen */
    return -1;
}

int CanIoBoxTx(void)
{
    const auto cfg = GetConfiguration();

    if (!cfg->iobox.enable_tx)
        return 0;

    if (!configured)
        return 0;

    const uint32_t base = CanIoBoxBaseId();

    if (1) {
        CanTxTyped<iobox_adc14> frame(base + CAN_IOBOX_ADC14, false);

        for (size_t i = 0; i < 4; i++) {
            frame.get().adc[i] = CanIoBoxGetAdc(i);
        }
    }
    if (1) {
        CanTxTyped<iobox_adc57> frame(base + CAN_IOBOX_ADC57, false);

        for (size_t i = 0; i < 3; i++) {
            frame.get().adc[i] = CanIoBoxGetAdc(4 + i);
        }
    }

    return 0;
}
