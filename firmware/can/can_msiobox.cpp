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

// MS IoBox protocol

#define MSIOBOX_TX_PERIOD_MS    10
#define MSIOBOX_BASE_ID         0x200
#define MSIOBOX_OFFSSET         0x20

namespace msiobox
{

// RX SYN: ID = BASE_ID + 0
// TX ACK: ID = BASE_ID + 8

// Settings: ID = BASE_ID + 1
struct IoBoxSettings
{
    uint8_t OutputMode;    // 0 = On/Off, 1 = PWM
    uint8_t : 8;           // Reserved
    uint8_t TachConfig;    // Bitfield
    uint8_t : 8;           // Reserved
    uint8_t AdcBroadcastInterval;  // ms
    uint8_t TachBroadcastInterval; // ms
    uint8_t : 16;          // Reserved
} __attribute__((packed));

static_assert(sizeof(IoBoxSettings) == 8);

// Output:
// ID = BASE_ID + 2 : Channel 1, 2
// ID = BASE_ID + 3 : Channel 3, 4
// ID = BASE_ID + 4 : Channel 5, 6
struct IoBoxOutputSettings
{
    struct {
        uint16_t OnPeriod;
        uint16_t OffPeriod;
    } __attribute__((packed)) Channels[2];
} __attribute__((packed));

static_assert(sizeof(IoBoxOutputSettings) == 8);

// Output:
// ID = BASE_ID + 5 : Channel 7, Output enables
struct IoBoxOutputState
{
    struct {
        uint16_t OnPeriod;
        uint16_t OffPeriod;
    } __attribute__((packed)) Channels[1];
    uint8_t OutputEnable; // Bitfield
    uint8_t : 24;         // Reserved
} __attribute__((packed));

static_assert(sizeof(IoBoxOutputState) == 8);

} //namespace msiobox

void SendMsIoBoxFormat(Configuration* configuration)
{
    (void)configuration;
}

void HandleMsIoBoxCanMessage(const CANRxFrame* msg, Configuration* configuration, struct CanStatusData* statusData)
{
    (void)msg;
    (void)configuration;
    (void)statusData;
}