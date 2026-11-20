#pragma once

#include <cstdint>

namespace wbo {
// Separate request/reply IDs prevent legacy ACKs from confirming a bitrate write.
// F042 reserved vector word at +0x20; keep in sync with bootloader/cfg/bootloader.ld.
constexpr uint32_t CanBitrateBootloaderMarker = 0x57424331;
constexpr uint32_t CanBitrateBootloaderMarkerAddress = 0x08000020;
constexpr uint32_t SetBitrateId = 0x0EF80000;
constexpr uint32_t BitrateAckId = 0x0EF80001;
enum class CanBitrate : uint8_t { Kbit500 = 0, Mbit1 = 1 };
constexpr bool IsCanBitrateValid(uint8_t value) { return value <= 1; }
constexpr bool IsBitrateChangeAllowed(uint8_t value, uint32_t bootloaderMarker) {
    return IsCanBitrateValid(value) && bootloaderMarker == CanBitrateBootloaderMarker;
}
constexpr bool IsBitrateRequestValid(uint8_t hwId, uint8_t value) {
    return hwId <= 8 && IsCanBitrateValid(value);
}
constexpr bool IsBitrateAck(uint32_t id, uint8_t dlc, uint8_t hwId, uint8_t value, uint8_t expectedHwId, uint8_t expectedValue) {
    return id == BitrateAckId && dlc == 2 && IsBitrateRequestValid(hwId, value) && hwId == expectedHwId && value == expectedValue;
}

// Stable ABI at the last 8 bytes of the F042 config page, independent of Configuration.
struct CanBitrateRecord {
    uint32_t magic;
    uint16_t value;
    uint16_t inverted;
};
constexpr uint32_t CanBitrateMagic = 0x57425231;
constexpr uint32_t CanBitrateRecordOffset = 1024 - sizeof(CanBitrateRecord);
static_assert(sizeof(CanBitrateRecord) == 8);
constexpr CanBitrateRecord MakeCanBitrateRecord(CanBitrate value) {
    return {CanBitrateMagic, static_cast<uint16_t>(value), static_cast<uint16_t>(~static_cast<uint16_t>(value))};
}
constexpr CanBitrate DecodeCanBitrate(const CanBitrateRecord& record) {
    return record.magic == CanBitrateMagic && record.value <= 1 &&
        static_cast<uint16_t>(record.value ^ record.inverted) == 0xffff
        ? static_cast<CanBitrate>(record.value) : CanBitrate::Kbit500;
}
} // namespace wbo
