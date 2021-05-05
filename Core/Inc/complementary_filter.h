#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "dim3.h"

typedef struct ComplementaryFilter_Config {
	float alpha;
	float sample_rate; // Hz
} ComplementaryFilter_Config;

typedef struct ComplementaryFilter {
	ComplementaryFilter_Config config;
	float roll, pitch, yaw;
	struct {
		float alpha;
		float one_minus_alpha;
		float sample_time; // s
		float g_roll;
	} _;
} ComplementaryFilter;

void ComplementaryFilter_Init(
		ComplementaryFilter *filter,
		ComplementaryFilter_Config config);
void ComplementaryFilter_Compute(
		ComplementaryFilter *filter,
		dim3_f xl, dim3_f g);

#endif /*  COMPLEMENTARY_FILTER_H */
