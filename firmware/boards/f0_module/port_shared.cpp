#include "port_shared.h"
#include "../../../for_rusefi/wideband_bitrate.h"

// Shared by application and bootloader. CAN1 is clocked from APB.
static_assert(STM32_PCLK1 == 48000000, "f0_module CAN timing requires 48 MHz APB");

static const CANConfig canConfig500 = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_BRP(5) | CAN_BTR_TS1(12) | CAN_BTR_TS2(1),
};
static const CANConfig canConfig1000 = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_BRP(2) | CAN_BTR_TS1(12) | CAN_BTR_TS2(1),
};

extern uint8_t __configflash__start__[];

wbo::CanBitrate ReadCanBitrate() {
    const auto* record = reinterpret_cast<const volatile wbo::CanBitrateRecord*>(
        __configflash__start__ + wbo::CanBitrateRecordOffset);
    const wbo::CanBitrateRecord copy{record->magic, record->value, record->inverted};
    return wbo::DecodeCanBitrate(copy);
}

const CANConfig& GetCanConfig() {
    // 16 TQ/bit, sample point 87.5%, SJW 1 TQ for both rates.
    return ReadCanBitrate() == wbo::CanBitrate::Mbit1 ? canConfig1000 : canConfig500;
}
