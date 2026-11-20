#!/usr/bin/env python3
"""Compile and exercise the real F0 bootloader/application CAN selector on the host."""
from pathlib import Path
import os
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

class CanBitrateTests(unittest.TestCase):
    def test_shared_flash_abi_and_can_selection(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / "hal.h").write_text("""#pragma once
#include <cstdint>
struct CANConfig { uint32_t mcr; uint32_t btr; };
#define STM32_PCLK1 48000000
#define CAN_MCR_ABOM 0x40
#define CAN_MCR_AWUM 0x20
#define CAN_MCR_TXFP 4
#define CAN_BTR_SJW(n) ((n) << 24)
#define CAN_BTR_BRP(n) (n)
#define CAN_BTR_TS1(n) ((n) << 16)
#define CAN_BTR_TS2(n) ((n) << 20)
""")
            (tmp / "test.cpp").write_text("""
#include "port_shared.h"
#include "wideband_bitrate.h"
#include "can_bitrate_storage.h"
#include <cassert>
#include <cstring>
#include <iostream>
alignas(4) uint8_t __configflash__start__[1024];
wbo::CanBitrate ReadCanBitrate();
static bool failWrite = false;
void Flash::ErasePage(uint8_t page) {
    assert(page == 31);
    std::memset(__configflash__start__, 0xff, 1024);
}
void Flash::Write(flashaddr_t addr, const uint8_t* data, size_t size) {
    if (!failWrite) {
        std::memcpy(reinterpret_cast<void*>(addr), data, size);
    }
}
static void install(const wbo::CanBitrateRecord& record) {
    std::memcpy(__configflash__start__ + wbo::CanBitrateRecordOffset, &record, sizeof(record));
}
int main() {
    using namespace wbo;
    std::memset(__configflash__start__, 0xff, sizeof(__configflash__start__));
    assert(ReadCanBitrate() == CanBitrate::Kbit500);
    assert(GetCanConfig().btr == 0x001c0005);
    for (auto rate : {CanBitrate::Kbit500, CanBitrate::Mbit1}) {
        auto record = MakeCanBitrateRecord(rate);
        install(record);
        assert(ReadCanBitrate() == rate);
        assert(GetCanConfig().btr == (rate == CanBitrate::Mbit1 ? 0x001c0002u : 0x001c0005u));
        assert(GetCanConfig().mcr == 0x64);
        // Ordinary configuration payload changes leave the shared trailer intact.
        std::memset(__configflash__start__, 0x42, 256);
        assert(ReadCanBitrate() == rate);
    }
    // Every single-bit corruption of a 1M record falls back to 500k.
    for (int bit = 0; bit < 64; ++bit) {
        auto record = MakeCanBitrateRecord(CanBitrate::Mbit1);
        reinterpret_cast<uint8_t*>(&record)[bit / 8] ^= 1 << (bit % 8);
        install(record);
        assert(ReadCanBitrate() == CanBitrate::Kbit500);
    }
    for (uint16_t value : {2, 255, 256, 65535}) {
        CanBitrateRecord record{CanBitrateMagic, value, uint16_t(~value)};
        install(record);
        assert(ReadCanBitrate() == CanBitrate::Kbit500);
    }
    // Simulated interrupted halfword writes never select 1M prematurely.
    auto record = MakeCanBitrateRecord(CanBitrate::Mbit1);
    for (unsigned count = 0; count < sizeof(record); count += 2) {
        std::memset(__configflash__start__, 0xff, 1024);
        std::memcpy(__configflash__start__ + CanBitrateRecordOffset, &record, count);
        assert(ReadCanBitrate() == CanBitrate::Kbit500);
    }
    uint8_t payload[256];
    std::memset(payload, 0x5a, sizeof(payload));
    for (auto rate : {CanBitrate::Mbit1, CanBitrate::Kbit500, CanBitrate::Mbit1}) {
        assert(WriteCanConfiguration(__configflash__start__, payload, sizeof(payload), rate));
        assert(ReadCanBitrate() == rate);
        assert(std::memcmp(__configflash__start__, payload, sizeof(payload)) == 0);
        // Sensor/index writes reuse the flash rate, including a pending rate.
        payload[10]++;
        assert(WriteCanConfiguration(__configflash__start__, payload, sizeof(payload), ReadCanBitrate()));
        assert(ReadCanBitrate() == rate);
    }
    failWrite = true;
    assert(!WriteCanConfiguration(__configflash__start__, payload, sizeof(payload), CanBitrate::Mbit1));
    assert(ReadCanBitrate() == CanBitrate::Kbit500);
    assert(!WriteCanConfiguration(__configflash__start__, payload, sizeof(payload), CanBitrate::Kbit500));
    failWrite = false;
    assert(!WriteCanConfiguration(__configflash__start__, payload, 1024, CanBitrate::Mbit1));
    assert(IsBitrateChangeAllowed(0, CanBitrateBootloaderMarker));
    assert(IsBitrateChangeAllowed(1, CanBitrateBootloaderMarker));
    assert(!IsBitrateChangeAllowed(2, CanBitrateBootloaderMarker));
    assert(!IsBitrateChangeAllowed(1, 0xffffffff));
    assert(!IsBitrateChangeAllowed(0, 0x08000191));
    assert(IsBitrateRequestValid(8, 1));
    assert(!IsBitrateRequestValid(0xff, 1));
    assert(!IsBitrateRequestValid(9, 0));
    assert(!IsBitrateRequestValid(1, 2));
    assert(IsBitrateAck(BitrateAckId, 2, 4, 1, 4, 1));
    assert(!IsBitrateAck(0x727573, 2, 4, 1, 4, 1));
    assert(!IsBitrateAck(BitrateAckId, 0, 4, 1, 4, 1));
    assert(!IsBitrateAck(BitrateAckId, 8, 4, 1, 4, 1));
    assert(!IsBitrateAck(BitrateAckId, 2, 5, 1, 4, 1));
    assert(!IsBitrateAck(BitrateAckId, 2, 4, 0, 4, 1));
    std::cout << "CAN selector, record corruption/torn writes and addressed ACK checks passed" << std::endl;
}
""")
            subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=undefined", "-DWB_PROD=1", "-I" + str(tmp),
                            "-I" + str(ROOT / "firmware/boards"), "-I" + str(ROOT / "for_rusefi"),
                            "-I" + str(ROOT / "firmware"), "-I" + str(ROOT / "firmware/boards/f0_module"),
                            str(ROOT / "firmware/boards/f0_module/port_shared.cpp"), str(tmp / "test.cpp"),
                            "-o", str(tmp / "test")], check=True)
            subprocess.run([str(tmp / "test")], check=True)
            # A changed clock must fail at compile time, in the actual shared source.
            hal = tmp / "hal.h"
            hal.write_text(hal.read_text().replace("48000000", "24000000"))
            bad_clock = subprocess.run([os.environ.get("CXX", "g++"), "-std=c++17", "-DWB_PROD=1",
                "-I" + str(tmp), "-I" + str(ROOT / "firmware/boards"),
                "-c", str(ROOT / "firmware/boards/f0_module/port_shared.cpp"),
                "-o", str(tmp / "bad-clock.o")], capture_output=True, text=True)
            self.assertNotEqual(bad_clock.returncode, 0)
            self.assertIn("f0_module CAN timing requires 48 MHz APB", bad_clock.stderr)

if __name__ == "__main__":
    unittest.main()
