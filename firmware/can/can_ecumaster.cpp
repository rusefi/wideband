#include "can.h"
#include "hal.h"

#include "util/byteswap.h"
#include "can_helper.h"
#include "can_ecumaster.h"

#include "port.h"
#include "status.h"
#include "can_helper.h"
#include "sampling.h"
#include "pump_dac.h"
#include "heater_control.h"
#include "lambda_conversion.h"
#include "max3185x.h"
#include "pwmout.h"

// EcuMaster protocol
// CAN 1Mbps, big-endian

#define ECUMASTER_L2C_TX_PERIOD_MS    10
#define ECUMASTER_L2C_BASE_ID         0x664

namespace ecumaster
{

enum class CalibrationStates : uint8_t {
    Start = 0,
    WaitForSPIReset = 1,
    Finished = 2,
    Error = 3,
};

enum class SensorType : uint8_t {
    LSU42 = 0,
    LSU49 = 1,
    LSUADV = 2,
};

// BASE_ID + 0
struct AfrData1
{
    beuint16_t SystemVolts;     // 0.00-655.35 V; 1/100
    uint8_t HeaterPower;        // 0-100 %; 100/255
    uint8_t SensorTemp;         // 0-1020 C; 4/1
    beuint16_t Lambda;          // 0.001 Lambda/bit, 0.000 to 65.535 Lambda
    uint8_t VmShortVcc : 1;
    uint8_t VmShortGnd : 1;
    uint8_t UnShortVcc : 1;
    uint8_t UnShortGnd : 1;
    uint8_t IaIpShortVcc : 1;
    uint8_t IaIpShortGnd : 1;
    uint8_t VubLowVoltage : 1;
    uint8_t HeaterShortVcc : 1;
    uint8_t HeaterShortGnd : 1;
    uint8_t HeaterOpenLoad : 1;
    CalibrationStates CalibrationState : 3;
    SensorType DeviceVersion : 3;
} __attribute__((packed));

static_assert(sizeof(AfrData1) == 8);

// BASE_ID + 1
struct AfrData2
{
    beint16_t IpCurrent;               // -32.768 to 32.767 mA; 1/1000
    beint16_t OxygenConcentration;     // -327.68 to 327.67 %; 1/100
    beuint16_t Ri;                     //  0.0 to 6553.5 Ohm; 1/10
    uint8_t Reserved[2];
} __attribute__((packed));

static_assert(sizeof(AfrData2) == 8);

} //namespace ecumaster


void SendEcuMasterAfrFormat(Configuration* configuration, uint8_t ch)
{
    auto id = ECUMASTER_L2C_BASE_ID + configuration->afr[ch].ExtraCanIdOffset * 2;
    const auto& sampler = GetSampler(ch);

    CanTxTyped<ecumaster::AfrData1> frame(id, true);
    frame->SystemVolts = sampler.GetInternalHeaterVoltage() * 100;
    frame->SensorTemp = sampler.GetSensorTemperature() / 4;
    frame->HeaterPower = GetHeaterDuty(ch) * 255;

    float lambda = GetLambda(ch);
    frame->Lambda = LambdaIsValid(ch, lambda) ? lambda * 1000 : 0;

    frame->CalibrationState = ecumaster::CalibrationStates::Finished;
    switch (configuration->sensorType) {
        case SensorType::LSU42:
            frame->DeviceVersion = ecumaster::SensorType::LSU42;
            break;
        case SensorType::LSU49:
            frame->DeviceVersion = ecumaster::SensorType::LSU49;
            break;
        case SensorType::LSUADV:
            frame->DeviceVersion = ecumaster::SensorType::LSUADV;
            break;
    }

    CanTxTyped<ecumaster::AfrData2> frame2(id + 1, true);
    frame2->IpCurrent = sampler.GetPumpNominalCurrent() * 1000;

    frame2->OxygenConcentration = GetOxygenConcentration(lambda) * 100;
    
    frame2->Ri = sampler.GetSensorInternalResistance() * 10;
}

constexpr ProtocolHandler ecuMasterAfrTxHandler = MakeProtocolHandler<&SendEcuMasterAfrFormat>(ECUMASTER_L2C_TX_PERIOD_MS);

#if (EGT_CHANNELS > 0)

#define ECU_MASTER_EGT_TX_PERIOD_MS    50

namespace ecumaster
{
    
// Offset: 0, 1
#define ECUMASTER_CLASSIC_EGT_BASE_ID         0x610
#define ECUMASTER_BLACK_EGT_BASE_ID           0x660

struct EgtData
{
    beint16_t Egt[4];
} __attribute__((packed));

static_assert(sizeof(EgtData) == 8);

} //namespace ecumaster


void SendEcuMasterEgtFormat(Configuration* configuration)
{
    auto base = ECUMASTER_CLASSIC_EGT_BASE_ID;
    if (configuration->egt[0].ExtraCanProtocol == CanEgtProtocol::EcuMasterBlack)
        base = ECUMASTER_BLACK_EGT_BASE_ID;

    auto id = base + configuration->egt[0].ExtraCanIdOffset;

    CanTxTyped<ecumaster::EgtData> frame(id, true);

    for (uint8_t i = 0; i < EGT_CHANNELS; i++)
    {
        if (!configuration->egt[i].ExtraCanChannelEnabled)
            continue;

        frame.get().Egt[i] = getEgtDrivers()[i].temperature;
    }
}

constexpr ProtocolHandler ecuMasterClassicEgtTxHandler = MakeProtocolHandler<&SendEcuMasterEgtFormat>(ECU_MASTER_EGT_TX_PERIOD_MS);
constexpr ProtocolHandler ecuMasterBlackEgtTxHandler = MakeProtocolHandler<&SendEcuMasterEgtFormat>(ECU_MASTER_EGT_TX_PERIOD_MS);

#endif


#if (IO_EXPANDER_ENABLED > 0)

#define ECUMASTER_SWITCHBOARD_TX_PERIOD_MS    100
#define ECUMASTER_SWITCHBOARD_BASE_ID         0x640
#define ECUMASTER_SWITCHBOARD_DEVICE_ID(n)    (ECUMASTER_SWITCHBOARD_BASE_ID + (n) * 4)
#define ECUMASTER_SWITCHBOARD_RX_OFFSET       3

namespace ecumaster
{
    // BASE:     AV1-AV4, 0.001 V
    // BASE + 1: AV5-AV8, 0.001 V
    struct AVData
    {
        uint16_t Value[4]; 
    } __attribute__((packed));

    static_assert(sizeof(AVData) == 8);

    // BASE + 2
    struct OtherData
    {
        uint8_t Rotary1 : 4;  // 0-15
        uint8_t Rotary2 : 4;  // 0-15
        uint8_t Rotary3 : 4;  // 0-15
        uint8_t Rotary4 : 4;  // 0-15
        uint8_t Rotary5 : 4;  // 0-15
        uint8_t Rotary6 : 4;  // 0-15
        uint8_t Rotary7 : 4;  // 0-15
        uint8_t Rotary8 : 4;  // 0-15
        uint8_t Switch;       // bit 0-7 represent switch 1-8 (0=open, 1=closed)
        uint8_t AnalogState;  // bitmask of which analog inputs are above threshold (1) or below threshold (0)
        uint8_t LowSideState; // bitmask of which low-side outputs are active (1) or inactive (0)
        uint8_t Heartbeat;    // Incremented on each message
    } __attribute__((packed));

    static_assert(sizeof(OtherData) == 8);

    // BASE + 3: Low-side output control messages from ECU to switchboard
    struct LowSideRxData
    {
        uint8_t State[4];
        uint8_t Reserved[4];
    } __attribute__((packed));

    static_assert(sizeof(LowSideRxData) == 8);
}

static uint8_t hearbeat = 0;
static uint8_t analogStatus = 0;
static uint8_t lowSideStatus = 0;

void SendEcuMasterSwitchBoardFormat(Configuration* configuration)
{

    auto id = ECUMASTER_SWITCHBOARD_BASE_ID + configuration->afr[0].ExtraCanIdOffset;
    
    // Handle first 4 channels for now
    constexpr uint8_t channels = AUX_INPUT_CHANNELS > 4 ? 4 : AUX_INPUT_CHANNELS;

    CanTxTyped<ecumaster::AVData> frame(id, true);
    for (uint8_t i = 0; i < channels; i++)
    {
        float voltage = GetAuxInputVoltage(i);

        // Some hysteresis
        if (voltage > 3.0f) analogStatus |= (1 << i);
        if (voltage < 2.0f) analogStatus &= ~(1 << i);

        uint16_t raw = (uint16_t)(voltage * 1000);
        frame->Value[i] = raw;
    }

    CanTxTyped<ecumaster::OtherData> otherFrame(id + 2, true);

    otherFrame->AnalogState = analogStatus;
    otherFrame->LowSideState = lowSideStatus;
    otherFrame->Heartbeat = hearbeat++;
}

void HandleEcuMasterCanMessage(const CANRxFrame* msg, Configuration* configuration)
{
    uint32_t msg_id = CAN_ID(*msg);
    uint32_t device_id = ECUMASTER_SWITCHBOARD_DEVICE_ID(configuration->ioExpanderConfig.Offset);

    if (msg_id != (device_id + ECUMASTER_SWITCHBOARD_RX_OFFSET)) {
        return; // Not a low-side control message
    }

    auto lowSideData = reinterpret_cast<const ecumaster::LowSideRxData*>(msg->data8);
    for (uint8_t i = 0; i < PWM_OUTPUT_CHANNELS; i++)
    {
        if (lowSideData->State[i])
        {
            lowSideStatus |= (1 << i);
            SetAuxPwmDuty(i, 1.0); // Set PWM duty to 100% for active low-side output
        }
        else
        {
            lowSideStatus &= ~(1 << i);
            SetAuxPwmDuty(i, 0.0); // Set PWM duty to 0% for inactive low-side output
        }
    }
}

constexpr ProtocolHandler ecuMasterSwitchBoardTxHandler = MakeProtocolHandler<&SendEcuMasterSwitchBoardFormat>(ECUMASTER_SWITCHBOARD_TX_PERIOD_MS);

#endif