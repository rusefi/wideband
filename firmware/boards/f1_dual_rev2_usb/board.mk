USE_BOOTLOADER = no
ENABLE_TS = yes

MCU = cortex-m3

ALLCPPSRC += $(BOARDDIR)/../f1_common/f1_port.cpp

# TunerStudio over USB CDC (OTG_FS)
ALLCSRC += $(BOARDDIR)/../f1_common/usbcfg.c
ALLINC += $(BOARDDIR)/../f1_common

include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f1xx.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F1xx/platform_f105_f107.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk

# platform_f105_f107.mk does not list embedded flash driver, needed for settings storage
ALLCSRC += $(CHIBIOS)/os/hal/ports/STM32/STM32F1xx/hal_efl_lld.c
