#pragma once

#include "shared/flash.h"
#include "../../../for_rusefi/wideband_bitrate.h"
// Volatile readback is required: flash programming can fail despite the CPU store.
inline bool VerifyCanFlash(const uint8_t* address, const uint8_t* expected, size_t size)
{
    const volatile uint8_t* flash = address;
    for (size_t i = 0; i < size; i++) {
        if (flash[i] != expected[i]) {
            return false;
        }
    }
    return true;
}

// Caller holds the system lock. Keep the record ABI independent of Configuration.
inline bool WriteCanConfiguration(uint8_t* page, const uint8_t* payload, size_t size, wbo::CanBitrate bitrate)
{
    if (size > wbo::CanBitrateRecordOffset || !wbo::IsCanBitrateValid(static_cast<uint8_t>(bitrate))) {
        return false;
    }
    const auto record = wbo::MakeCanBitrateRecord(bitrate);
    Flash::ErasePage(31);
    Flash::Write(reinterpret_cast<flashaddr_t>(page), payload, size);
    Flash::Write(reinterpret_cast<flashaddr_t>(page) + wbo::CanBitrateRecordOffset,
        reinterpret_cast<const uint8_t*>(&record), sizeof(record));
    // Verify payload and record before acknowledging the command.
    return VerifyCanFlash(page, payload, size) &&
        VerifyCanFlash(page + wbo::CanBitrateRecordOffset,
            reinterpret_cast<const uint8_t*>(&record), sizeof(record));
}
