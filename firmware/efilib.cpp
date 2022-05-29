#include "ch.h"
#include "osal.h"

int minI(int i1, int i2) {
	return i1 < i2 ? i1 : i2;
}

extern "C" void __cxa_pure_virtual(void) {
	osalSysHalt("Pure virtual function call.");
}
