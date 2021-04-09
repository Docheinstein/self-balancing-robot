#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#include "dim3.h"

typedef struct MadgwickFilter_Config {
	float beta;
	float sample_rate; // Hz
} MadgwickFilter_Config;

void MadgwickFilter_Init(MadgwickFilter_Config config);
void MadgwickFilter_Compute(dim3_f xl, dim3_f g, float *roll, float *pitch, float *yaw);

#endif /*  MADGWICK_FILTER_H */
