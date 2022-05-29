#include "livedata.h"

#include "lambda_conversion.h"
#include "sampling.h"
#include "pump_dac.h"

volatile struct livedata_common_s livedata_common;
volatile struct livedata_afr_s livedata_afr;

void SamplingUpdateLiveData()
{
    livedata_afr.afr = GetLambda();
    livedata_afr.temperature = (GetSensorTemperature(0) + GetSensorTemperature(1)) / 2;
    livedata_afr.nernstVoltage = GetNernstDc();
    livedata_afr.pumpCurrentTarget = GetPumpCurrent();
    livedata_afr.pumpCurrentMeasured = GetPumpNominalCurrent();

    livedata_common.vbatt = GetInternalBatteryVoltage();
}
