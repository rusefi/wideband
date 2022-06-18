#include "wideband_config.h"
#include "livedata.h"

#include "lambda_conversion.h"
#include "sampling.h"
#include "pump_dac.h"
#include "heater_control.h"

volatile struct livedata_common_s livedata_common;
volatile struct livedata_afr_s livedata_afr[AFR_CHANNELS];

void SamplingUpdateLiveData()
{
    for (int ch = 0; ch < AFR_CHANNELS; ch++)
    {
        volatile struct livedata_afr_s *data = &livedata_afr[ch];

        data->afr = GetLambda(ch);
        data->temperature = GetSensorTemperature(ch);
        data->nernstVoltage = GetNernstDc(ch);
        data->pumpCurrentTarget = GetPumpCurrent(ch);
        data->pumpCurrentMeasured = GetPumpNominalCurrent(ch);
        data->heaterDuty = GetHeaterDuty(ch);
    }

    livedata_common.vbatt = GetInternalBatteryVoltage(0);
}
