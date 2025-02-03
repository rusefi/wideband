#!/bin/sh

LD_PRELOAD=/usr/local/lib/libjaylink.so openocd -f interface/jlink.cfg -c 'transport select swd' -f target/stm32f1x.cfg

#-c '$_TARGETNAME configure -rtos chibios'
