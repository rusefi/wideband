#pragma once

// Used by rusefi/timer
// We are using ChibiOS's chVTGetTimeStamp() not raw high frequency free-runnig timer
// See getTimeNowNt()
#define US_TO_NT_MULTIPLIER (1)
