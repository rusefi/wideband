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
#include "can/can_msiobox.h"

#include "port.h"

#include <cstddef>


static Configuration* configuration;

static struct CanStatusData canStatusData = {
    .heaterAllow = HeaterAllow::Unknown,
    .remoteBatteryVoltage = 0.0f,
};


__attribute__((weak)) void SendCanData(uint16_t elapsedMs)
{
    #if (AFR_CHANNELS > 0)
    // f0 memory is tight, so we keep this as an array of timers
    // otherwise padding eats up more ram
    static uint16_t elapsedSinceAfrTxMs[AFR_CHANNELS * 2] = {0};

    for (size_t i = 0; i < AFR_CHANNELS; i++) {
        DispatchProtocolHandler(rusefiAfrTxHandler, elapsedMs, elapsedSinceAfrTxMs[i], configuration, i);
        const ProtocolHandler* extraHandler = nullptr;

        switch (configuration->afr[i].ExtraCanProtocol) {
            case CanAfrProtocol::AemNet:
                extraHandler = &aemNetAfrTxHandler;
                break;
            case CanAfrProtocol::EcuMaster:
                extraHandler = &ecuMasterAfrTxHandler;
                break;
            case CanAfrProtocol::Emtron:
                extraHandler = &emtronAfrTxHandler;
                break;
            case CanAfrProtocol::Motec:
                extraHandler = &motecAfrTxHandler;
                break;
            case CanAfrProtocol::LinkEcu:
                extraHandler = &linkAfrTxHandler;
                break;
            default:
                break;
        }
        if (extraHandler) {
            DispatchProtocolHandler(*extraHandler, elapsedMs, elapsedSinceAfrTxMs[i + AFR_CHANNELS], configuration, i);
        }
    }

    // Handle Haltech separately since it sends both AFR channels in the same message
    if (configuration->afr[0].ExtraCanProtocol == CanAfrProtocol::Haltech
        || configuration->afr[1].ExtraCanProtocol == CanAfrProtocol::Haltech) {
        DispatchProtocolHandler(haltechAfrTxHandler, elapsedMs, elapsedSinceAfrTxMs[0], configuration);
    }

    #endif

    #if (EGT_CHANNELS > 0)

    static uint16_t elapsedSinceEgtTxMs = 0;
    const ProtocolHandler* egtHandler = nullptr;
    switch (configuration->egt[0].ExtraCanProtocol) {
        case CanEgtProtocol::AemNet0305:
            egtHandler = &aemNet0305EgtTxHandler;
            break;
        case CanEgtProtocol::AemNet2224:
            egtHandler = &aemNet2224EgtTxHandler;
            break;
        case CanEgtProtocol::EcuMasterClassic:
            egtHandler = &ecuMasterClassicEgtTxHandler;
            break;
        case CanEgtProtocol::EcuMasterBlack:
            egtHandler = &ecuMasterBlackEgtTxHandler;
            break;
        case CanEgtProtocol::Haltech:
            egtHandler = &haltechEgtTxHandler;
            break;
        case CanEgtProtocol::LinkEcu:
            egtHandler = &linkEgtTxHandler;
            break;
        case CanEgtProtocol::Emtron:
            egtHandler = &emtronEgtTxHandler;
            break;
        default:
            break;
    }
    if (egtHandler) {
        DispatchProtocolHandler(*egtHandler, elapsedMs, elapsedSinceEgtTxMs, configuration);
    }

    #endif

    #if (IO_EXPANDER_ENABLED > 0)

    static uint16_t elapsedSinceIoExpanderTxMs = 0;
    const ProtocolHandler* ioExpanderHandler = nullptr;
    if (configuration->ioExpanderConfig.TxEnabled) {
        switch (configuration->ioExpanderConfig.Protocol) {
            case CanIoProtocol::EcuMaster:
                ioExpanderHandler = &ecuMasterSwitchBoardTxHandler;
                break;
            case CanIoProtocol::Haltech:
                ioExpanderHandler = &haltechIoTxHandler;
                break;
            case CanIoProtocol::Emtron:
                ioExpanderHandler = &emtronIoTxHandler;
                break;
            case CanIoProtocol::MsIoBox:
                ioExpanderHandler = &msIoBoxTxHandler;
                break;
            default:
                break;
        }
        if (ioExpanderHandler) {
            DispatchProtocolHandler(*ioExpanderHandler, elapsedMs, elapsedSinceIoExpanderTxMs, configuration);
        }
    } 

    #endif

    // E888 combines EGT and IO expander data, so handle it separately
    #if (MOTEC_E888_ENABLED > 0)
        static uint16_t elapsedSinceMotecE888TxMs = 0;
        if (IsMotecE888Enabled(configuration)) {
            DispatchProtocolHandler(motecE888TxHandler, elapsedMs, elapsedSinceMotecE888TxMs, configuration);
        }
    #endif
}

__attribute__((weak)) void ProcessCanMessage(const CANRxFrame* frame)
{
    ProcessRusefiCanMessage(frame, configuration, &canStatusData);

    //ProcessLinkCanMessage(frame, configuration, &canStatusData);

    #if IO_EXPANDER_ENABLED > 0
    if (configuration->ioExpanderConfig.RxEnabled) {
        switch (configuration->ioExpanderConfig.Protocol) {
            case CanIoProtocol::Haltech:
                ProcessHaltechIO12Message(frame, configuration);
                break;
            case CanIoProtocol::EcuMaster:
                HandleEcuMasterCanMessage(frame, configuration);
                break;
            case CanIoProtocol::Motec:
                // TODO: Implement Motec E888 RX message processing
                break;
            default:
                break;
        }
    }

    // IOBox needs to config packages to know if it should send data
    // We handle disabling output control in the handler instead of here
    if (configuration->ioExpanderConfig.Protocol == CanIoProtocol::MsIoBox) {
        ProcessMsIoBoxCanMessage(frame, configuration);
    }

    #endif
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