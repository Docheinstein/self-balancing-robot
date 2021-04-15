#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include "dim3.h"

typedef struct ComplementaryFilter_Config {
	float alpha;
	float sample_rate; // Hz
} ComplementaryFilter_Config;

void ComplementaryFilter_Init(ComplementaryFilter_Config config);
void ComplementaryFilter_Compute(dim3_f xl, dim3_f g, float *roll, float *pitch, float *yaw);

#endif /*  COMPLEMENTARY_FILTER_H */