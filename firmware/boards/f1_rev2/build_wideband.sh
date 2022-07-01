#!/bin/bash

set -e

# back out to the root
cd ../..

export EXTRA_PARAMS="-DECHO_UART=TRUE"
# set optimization level to 0 until ADC issue is fixed for GD32
export USE_OPT="-O0 -ggdb -fomit-frame-pointer -falign-functions=16 -fsingle-precision-constant"

# build app firmware!
make -j12 BOARD=f1_rev2
