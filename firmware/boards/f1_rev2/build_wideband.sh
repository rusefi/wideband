#!/bin/bash

set -e

BOARD=f1_rev2

cd openblt

echo ""
echo "Building bootloader"
#make clean
make -j12 BOARD=${BOARD} || exit 1

# back out to the root
cd ../../..

echo ""
echo "Build application"
export EXTRA_PARAMS="-DECHO_UART=TRUE"
#make clean
make -j12 BOARD=${BOARD} || exit 1

echo ""
echo "Creating deliveries:"

if uname | grep "NT"; then
  HEX2DFU=./ext/encedo_hex2dfu/hex2dfu.exe
else
  HEX2DFU=./ext/encedo_hex2dfu/hex2dfu.bin
fi
chmod u+x $HEX2DFU

DELIVER_DIR=deliver/${BOARD}

mkdir -p ${DELIVER_DIR}
rm -f ${DELIVER_DIR}/*

echo "Srec for CAN update"
cp -v build/wideband.srec ${DELIVER_DIR}/wideband_update.srec

echo ""
echo "Invoking hex2dfu for incremental Wideband image (for DFU util)"
$HEX2DFU -i build/wideband.hex -C 0x1C -o ${DELIVER_DIR}/wideband_update.dfu

echo ""
echo "Invoking hex2dfu for OpenBLT (for DFU util)"
$HEX2DFU -i boards/${BOARD}/openblt/bin/openblt_${BOARD}.hex -o ${DELIVER_DIR}/openblt.dfu

echo ""
echo "Invoking hex2dfu for composite OpenBLT+Wideband image (for DFU util)"
$HEX2DFU -i boards/${BOARD}/openblt/bin/openblt_${BOARD}.hex -i build/wideband.hex -C 0x1C -o ${DELIVER_DIR}/wideband.dfu -b ${DELIVER_DIR}/wideband.bin

echo ""
echo "${DELIVER_DIR} folder content:"
ls -l ${DELIVER_DIR}
