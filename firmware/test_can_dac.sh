#!/bin/bash

INTERFACE=can0
# base 0x200, base+2 = PWM1,2 periods packet
CANID=202
#See http://www.msextra.com/doc/pdf/Microsquirt-IObox-1.pdf for protocol specification
# Note BIG-endian values

while true; do
	# zero
	cansend $INTERFACE ${CANID}#0000.ffff.0000.ffff
	sleep 1
	# half
	cansend $INTERFACE ${CANID}#ff7f.ff7f.ff7f.ff7f
	sleep 1
	#full
	cansend $INTERFACE ${CANID}#ffff.0000.ffff.0000
	sleep 1
done
