#pragma once


#include <cstdint>

#include "hal.h"
#include "base_protocol_handler.h"
#include "port.h"

// E888 can handle both EGT and IO data
#if ((EGT_CHANNELS > 0) || (IO_EXPANDER_ENABLED > 0))
    #define MOTEC_E888_ENABLED 1
#endif

extern AfrHandler motecAfrTxHandler;
extern CallbackHandler motecE888TxHandler;