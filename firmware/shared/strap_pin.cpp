#include "hal.h"
#include "strap_pin.h"

// Returns:
// low -> 0
// floating -> 1
// high -> 2
uint8_t readSelPin(ioportid_t port, iopadid_t pad)
{
    // If we pull the pin down, does the input follow?
    palSetPadMode(port, pad, PAL_MODE_INPUT_PULLDOWN);
    chThdSleepMilliseconds(1);
    auto pd = palReadPad(port, pad);

    // If we pull the pin up, does the input follow?
    palSetPadMode(port, pad, PAL_MODE_INPUT_PULLUP);
    chThdSleepMilliseconds(1);
    auto pu = palReadPad(port, pad);

    // If the pin changed with pullup/down state, it's floating
    if (pd != pu)
    {
        return 1;
    }

    if (pu)
    {
        // Pin was high
        return 2;
    }
    else
    {
        // Pin was low
        return 0;
    }
}
