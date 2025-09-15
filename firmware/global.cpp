#include "global.h"
#include "hal.h" // OSAL_ST_FREQUENCY
#include "ch.hpp" // chVTGetTimeStamp
#include <rusefi/rusefi_time_math.h>

efitick_t getTimeNowNt() {
    // chVTGetTimeStamp() has 1 / OSAL_ST_FREQUENCY resolution
    // See CH_CFG_ST_FREQUENCY in board configs
    // convert to uS and then to ticks
    return USF2NT(1000000 / OSAL_ST_FREQUENCY * chVTGetTimeStamp());
}

efitimeus_t getTimeNowUs() {
    return NT2US(getTimeNowNt());
}
