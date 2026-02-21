#include "ch.h"
#include "hal.h"

#include "can.h"

#include "status.h"
#include "can_helper.h"
#include "can/base_protocol_handler.h"
#include "can/can_rusefi.h"
#include "can/can_aemnet.h"
#include "can/can_ecumaster.h"
#include "can/can_haltech.h"
#include "can/can_link.h"
#include "can/can_emtron.h"
#include "can/can_motec.h"

#include "port.h"

#include <cstddef>


static Configuration* configuration;

static struct CanStatusData canStatusData = {
    .heaterAllow = HeaterAllow::Unknown,
    .remoteBatteryVoltage = 0.0f,
};

static BaseProtocolHandler* const TxHandlers[] = {
    #if (AFR_CHANNELS > 0)
    &rusefiAfrTxHandler,
    &aemNetAfrTxHandler,
    &ecuMasterAfrTxHandler,
    &haltechAfrTxHandler,
    &emtronAfrTxHandler,
    &motecAfrTxHandler,
    &linkAfrTxHandler,
    #endif

    #if (EGT_CHANNELS > 0)
    &aemNet0305EgtTxHandler,
    &aemNet2224EgtTxHandler,
    &ecuMasterClassicEgtTxHandler,
    &ecuMasterBlackEgtTxHandler,
    &haltechEgtTxHandler,
    &linkEgtTxHandler,
    &emtronEgtTxHandler,
    #endif

    #if (IO_EXPANDER_ENABLED > 0)
    &haltechIoTxHandler,
    &emtronIoTxHandler,
    #endif

    #if (MOTEC_E888_ENABLED > 0)
    &motecE888TxHandler,
    #endif
};

__attribute__((weak)) void SendCanData(uint32_t elapsedMs)
{
    for (auto* handler : TxHandlers) {
        handler->dispatch(elapsedMs, configuration);
    }
}

__attribute__((weak)) void ProcessCanMessage(const CANRxFrame* frame)
{
    ProcessRusefiCanMessage(frame, configuration, &canStatusData);
    ProcessLinkCanMessage(frame, configuration, &canStatusData);
    ProcessHaltechIO12Message(frame, configuration);
}

static THD_WORKING_AREA(waCanTxThread, 512);
void CanTxThread(void*)
{
    chRegSetThreadName("CAN Tx");

    // Current system time.
    systime_t prev = chVTGetSystemTime();
    uint32_t prevMs = TIME_I2MS(prev);

    while(1)
    {
        uint32_t nowMs = TIME_I2MS(chVTGetSystemTime());
        uint32_t elapsedMs = nowMs - prevMs;
        prevMs = nowMs;
        SendCanData(elapsedMs);

        prev = chThdSleepUntilWindowed(prev, chTimeAddX(prev, TIME_MS2I(WBO_TX_PERIOD_MS)));
    }
}

static THD_WORKING_AREA(waCanRxThread, 512);
void CanRxThread(void*)
{
    chRegSetThreadName("CAN Rx");

    while(1)
    {
        CANRxFrame frame;
        msg_t msg = canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_INFINITE);

        // Ignore non-ok results...
        if (msg != MSG_OK)
        {
            continue;
        }

        ProcessCanMessage(&frame);
    }
}

HeaterAllow GetHeaterAllowed()
{
    return canStatusData.heaterAllow;
}

float GetRemoteBatteryVoltage()
{
    return canStatusData.remoteBatteryVoltage;
}

void InitCan()
{
    configuration = GetConfiguration();

    canStart(&CAND1, &GetCanConfig(configuration->CanMode));
    chThdCreateStatic(waCanTxThread, sizeof(waCanTxThread), NORMALPRIO, CanTxThread, nullptr);
    chThdCreateStatic(waCanRxThread, sizeof(waCanRxThread), NORMALPRIO - 4, CanRxThread, nullptr);
}