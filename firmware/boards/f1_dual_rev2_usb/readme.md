Firmware for dual channel board rev2 with USB: based on [f1_dual_rev1](../f1_dual_rev1), MCU is STM32F105RC.

Differences from f1_dual_rev1:

| | f1_dual_rev1 | f1_dual_rev2_usb |
|---|---|---|
| MCU | STM32F103RC | STM32F105RC (256k flash, 64k RAM) |
| TunerStudio primary port | Bluetooth (USART3) | USB CDC (OTG_FS, PA11 - DM, PA12 - DP) |
| TunerStudio secondary port | UART on J3 (USART1, PA9/PA10) | Bluetooth (USART3) |
| CAN1 | PA11 (RX), PA12 (TX) | PB8 (RX), PB9 (TX) - remap, PA11/PA12 are USB |
| AUX outputs PWM, PC6..PC9 | TIM8 | TIM3 full remap (no TIM8 on F105) |
| System tick timer | TIM3 | TIM2 |
| OpenBLT | CAN + UART | CAN only |

USB VBUS is not sensed (`BOARD_OTG_NOVBUSSENS` in board.h), PA9 and PA10 are not used.
