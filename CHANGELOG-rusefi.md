# rusEFI wideband: changes on top of upstream

previous PROD branch https://github.com/rusefi/wideband/tree/prod-2026-01

https://github.com/rusefi/wideband/blob/prod-2026-01/CHANGELOG-rusefi.md

## Build, CI, and release

- ARM toolchain updated from 11.3.rel1 to 14.3.rel1.
- F0 image header includes the build date and time; `build_defs.h` exposes the build date as integers.
