#pragma once

#include <stdint.h>

/* +0 offset */
struct livedata_common_s {
	union {
		struct {
			uint32_t test;
			float vbatt;
		};
		uint8_t pad0[32];
	};
};

extern volatile struct livedata_common_s livedata_common;

/* +32 offset */
struct livedata_afr_s {
	union {
		struct {
			float afr;
			float temperature;
			float nernstVoltage;
			float pumpCurrentTarget;
			float pumpCurrentMeasured;
		};
		uint8_t pad[32];
	};
};

extern volatile struct livedata_afr_s livedata_afr;

/* update functions */
void SamplingUpdateLiveData();