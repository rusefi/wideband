#pragma once

#include "io_pins.h"
#include "wideband_board_config.h"

// *******************************
//    Defaults
// *******************************
#ifndef EGT_CHANNELS
    #define EGT_CHANNELS 0
#endif

// *******************************
//    Nernst voltage & ESR sense
// *******************************

// Heater low pass filter
#define ESR_SENSE_ALPHA (0.002f)

// *******************************
//       Pump current sense
// *******************************

#define PUMP_CURRENT_SENSE_GAIN (10)

// LSU sense resistor - 61.9 ohms
#define LSU_SENSE_R (61.9f)

// Pump low pass filter alpha
// sampling at 2.5khz, alpha of 0.01 gives about 50hz bandwidth
#define PUMP_FILTER_ALPHA (0.02f)

// *******************************
//        Pump controller
// *******************************
#define NERNST_TARGET (0.45f)

// *******************************
//    Heater controller config
// *******************************
#define HEATER_CONTROL_PERIOD 50

#define HEATER_PREHEAT_TIME 5
#define HEATER_WARMUP_TIMEOUT 60

// time to wait after heater voltage rise above HEATER_SUPPLY_ON_VOLTAGE
#define HEATER_SUPPLY_STAB_TIME 0.5f
// minimal heater voltage to start heating without CAN command
#define HEATER_SUPPLY_ON_VOLTAGE 9.5
// mininal heater voltage to continue heating
#define HEATER_SUPPLY_OFF_VOLTAGE 8.5

// *******************************
// Start driving the pump just before we're at target temperature
// minus this offset to avoid Vnerns voltage clamp near 0V
// *******************************
#define START_PUMP_TEMP_OFFSET	(200.0)


// *******************************
//    CAN config
// *******************************
// default interval is 50mS
#define CAN_STATUS_MSG_TIMEOUT_MS   250
