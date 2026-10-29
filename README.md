[![Build Firmware](https://github.com/mck1117/wideband/actions/workflows/build-firmware.yaml/badge.svg)](https://github.com/mck1117/wideband/actions/workflows/build-firmware.yaml) ![license](https://img.shields.io/github/license/mck1117/wideband)

# rusEFI Wideband Controller

[User Documentation](https://rusefi.com/s/wb)

[Forum Thread](https://rusefi.com/forum/viewtopic.php?f=4&t=1856)

## Building Firmware

The `firmware/boards` directory contains configuration for each board this firmware supports.

For the standalone board and the module built in to rusEFI Hellen boards, `f0_module` is the correct target.  Use the `build_wideband.sh` script to build and package the wideband firmware: both a bin including the bootloader, and a header file consumed by rusEFI that contains no bootloader, to be uploaded over CAN (via the aforementioned bootloader).

## F0 module CAN bitrate

The configurable F0 firmware uses 500 kbit/s when its config-page bitrate record is absent or invalid. Both the bootloader and application read the same record; 500 kbit/s and 1 Mbit/s are supported. The 48 MHz APB clock is checked at compile time.

Send extended data frame `0x0EF80000`, DLC 2, `[hardware ID, rate]` (rate 0 = 500k, 1 = 1M). The addressed module stores the setting and replies on extended ID `0x0EF80001`, DLC 2, with the same bytes only after verifying the write. Its running bitrate does not change until the next reset. Configure all modules at the old rate, then match the ECU/adapter bitrate and restart them together. IDs must be unique. Do not enter the bootloader between storing the setting and matching the ECU bitrate.

The stable eight-byte record at `0x08007ff8` is independent of the application's Configuration version. Ordinary F0 config writes preserve it. A power loss during the shared config-page erase/write may lose settings and cause both parts to fall back to 500k. The application refuses bitrate writes unless the installed bootloader has the matching capability marker in its reserved vector word at 0x08000020. Initial migration requires SWD installation of both new bootloader and application; application-only CAN updates cannot replace a fixed-rate bootloader. Older firmware can erase the record and does not support this protocol.

Run `python3 test/test_can_bitrate.py` (host g++ with UBSan) to test the real CAN selector and flash writer with a mocked config page, invalid/torn records, write failures and addressed response matching. Hardware CAN timing, reset/update behavior and power-loss recovery still need bench validation.
