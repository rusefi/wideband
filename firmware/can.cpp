#include "hal.h"

#include "can.h"

#include "fault.h"
#include "can_helper.h"
#include "heater_control.h"
#include "lambda_conversion.h"
#include "sampling.h"
#include "pump_dac.h"
#include "port.h"

#include "build_defs.h"

// this same header is imported by rusEFI to get struct layouts and firmware version
#include "../for_rusefi/wideband_can.h"

using namespace wbo;

static Configuration* configuration;

static THD_WORKING_AREA(waCanTxThread, 512);
void CanTxThread(void*)
{
    chRegSetThreadName("CAN Tx");

    // Current system time.
    systime_t prev = chVTGetSystemTime();

    while(1)
    {
        for (int ch = 0; ch < AFR_CHANNELS; ch++)
        {
            SendCanForChannel(ch);
        }

        prev = chThdSleepUntilWindowed(prev, chTimeAddX(prev, TIME_MS2I(WBO_TX_PERIOD_MS)));
    }
}

static void SendAck()
{
    CANTxFrame frame;

#ifdef STM32G4XX
    frame.common.RTR = 0;
#else // Not CAN FD
    frame.RTR = CAN_RTR_DATA;
#endif

    CAN_EXT(frame) = 1;
    CAN_EID(frame) = WB_ACK;
    frame.DLC = 0;

    canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_INFINITE);
}

static void SendPong()
{
    CanTxTyped<wbo::PongData> frame(WB_ACK, true);;

    frame.get().hwId = BoardGetHwId();
    frame.get().Version = RUSEFI_WIDEBAND_VERSION;

    // Compile date
    frame.get().year = BUILD_YEAR_SINCE_2000;
    frame.get().month = BUILD_MONTH;
    frame.get().day = BUILD_DAY;
}

// Start in Unknown state. If no CAN message is ever received, we operate
// on internal battery sense etc.
static HeaterAllow heaterAllow = HeaterAllow::Unknown;
static float remoteBatteryVoltage = 0;
Timer canStatusMsgTimer;

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

        // Ignore std frames, only listen to ext
        if (!CAN_EXT(frame))
        {
            continue;
        }

        // Ignore not ours frames
        if (WB_MSG_GET_HEADER(CAN_ID(frame)) != WB_BL_HEADER)
        {
            continue;
        }

        if (frame.DLC == 2 && CAN_ID(frame) == WB_MGS_ECU_STATUS)
        {
            // This is status from ECU - battery voltage and heater enable signal

            // data1 contains heater enable bit
            if ((frame.data8[1] & 0x1) == 0x1)
            {
                heaterAllow = HeaterAllow::Allowed;
            }
            else
            {
                heaterAllow = HeaterAllow::NotAllowed;
            }

            // data0 contains battery voltage in tenths of a volt
            remoteBatteryVoltage = frame.data8[0] * 0.1f;
            canStatusMsgTimer.reset();
        }
        // If it's a bootloader entry request, reboot to the bootloader!
        else if ((frame.DLC == 0 || frame.DLC == 1) && CAN_ID(frame) == WB_BL_ENTER)
        {
            // If 0xFF (force update all) or our ID, reset to bootloader, otherwise ignore
            if (frame.DLC == 0 || frame.data8[0] == 0xFF || frame.data8[0] == BoardGetHwId())
            {
                SendAck();

                // Let the message get out before we reset the chip
                chThdSleep(50);

                NVIC_SystemReset();
            }
        }
        // Check if it's an "index set" message
        // Old style with DLC = 1, just set this index
        // New style with DLC = 2, set if byte[1] equals to your hardware ID
        else if (CAN_ID(frame) == WB_MSG_SET_INDEX &&
            ((frame.DLC == 1) || (frame.DLC == 2 && frame.data8[1] == BoardGetHwId())))
        {
            // new index
            int offset = frame.data8[0];
            configuration = GetConfiguration();
            for (int i = 0; i < AFR_CHANNELS; i++) {
                configuration->afr[i].RusEfiIdx = offset + i;
            }
            for (int i = 0; i < EGT_CHANNELS; i++) {
                configuration->egt[i].RusEfiIdx = offset + i;
            }
            SetConfiguration();
            SendAck();
        }
        else if (CAN_ID(frame) == WB_MSG_PING && frame.DLC == 1)
        {
            // broadcast or with our HW ID
            if (frame.data8[0] == 0xFF || frame.data8[0] == BoardGetHwId())
            {
                SendPong();
            }
        }
    }
}

HeaterAllow GetHeaterAllowed()
{
    if (!canStatusMsgTimer.hasElapsedMs(CAN_STATUS_MSG_TIMEOUT_MS))
    {
        return heaterAllow;
    }

    return HeaterAllow::Unknown;
}

float GetRemoteBatteryVoltage()
{
    if (!canStatusMsgTimer.hasElapsedMs(CAN_STATUS_MSG_TIMEOUT_MS))
    {
        return remoteBatteryVoltage;
    }

    return 0;
}

void InitCan()
{
    configuration = GetConfiguration();

    canStart(&CAND1, &GetCanConfig());
    chThdCreateStatic(waCanTxThread, sizeof(waCanTxThread), NORMALPRIO, CanTxThread, nullptr);
    chThdCreateStatic(waCanRxThread, sizeof(waCanRxThread), NORMALPRIO - 4, CanRxThread, nullptr);
}

void SendRusefiFormat(uint8_t ch)
{
    auto baseAddress = WB_DATA_BASE_ADDR + 2 * configuration->afr[ch].RusEfiIdx;

    const auto& sampler = GetSampler(ch);
    const auto& heater = GetHeaterController(ch);

    auto nernstDc = sampler.GetNernstDc();
    auto pumpDuty = GetPumpOutputDuty(ch);
    auto lambda = GetLambda(ch);

    // Lambda is valid if:
    // 1. Nernst voltage is near target
    // 2. Pump driver isn't slammed in to the stop
    // 3. Lambda is >0.6 (sensor isn't specified below that)
    bool lambdaValid =
            nernstDc > (NERNST_TARGET - 0.1f) && nernstDc < (NERNST_TARGET + 0.1f) &&
            pumpDuty > 0.1f && pumpDuty < 0.9f &&
            lambda > 0.6f;

    if (configuration->afr[ch].RusEfiTx) {
        CanTxTyped<wbo::StandardData> frame(baseAddress + 0);

        // The same header is imported by the ECU and checked against this data in the frame
        frame.get().Version = RUSEFI_WIDEBAND_VERSION;

        // Show calculated lambda even it is not valid. User should check Valid flag
        frame.get().Lambda = lambda * 10000;
        frame.get().TemperatureC = sampler.GetSensorTemperature();
        bool heaterClosedLoop = heater.IsRunningClosedLoop();
        frame.get().Valid = (heaterClosedLoop && lambdaValid) ? 0x01 : 0x00;
    }

    if (configuration->afr[ch].RusEfiTxDiag) {
        CanTxTyped<wbo::DiagData> frame(baseAddress + 1);;

        frame.get().Esr = sampler.GetSensorInternalResistance();
        frame.get().NernstDc = nernstDc * 1000;
        frame.get().PumpDuty = pumpDuty * 255;
        frame.get().Status = (GetHeaterAllowed() != HeaterAllow::Allowed) ?
            Fault::NotAllowed : GetCurrentFault(ch);
        frame.get().HeaterDuty = GetHeaterDuty(ch) * 255;
    }
}

// Weak link so boards can override it
__attribute__((weak)) void SendCanForChannel(uint8_t ch)
{
    SendRusefiFormat(ch);
}
