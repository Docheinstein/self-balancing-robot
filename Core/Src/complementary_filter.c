#include "complementary_filter.h"
#include "math.h"
#include "std.h"
#include "verbose.h"
#include "serial.h"

#define VERBOSE_FMT(fmt) "{COMPL_FILTER} " fmt

static float alpha;
static float one_minus_alpha;
static float sample_time;
static float roll;

void ComplementaryFilter_Init(ComplementaryFilter_Config config)
{
	verboseln(
		"Initialized" SERIAL_ENDL
		"- alpha: %f" SERIAL_ENDL
		"- sample_rate: %.2f",
		config.alpha,
		config.sample_rate
	);

	alpha = config.alpha;
	one_minus_alpha = 1 - alpha;
	sample_time = 1 / config.sample_rate;
	roll = NAN;
}

void ComplementaryFilter_Compute(dim3_f xl, dim3_f g, float *r, float *p, float *y)
{

	float xl_roll = RAD_TO_DEG(atan2f(xl.y, xl.z));
	if (isnanf(roll)) // just first time
		roll = xl_roll;
	roll = alpha * (roll + g.x * sample_time) + one_minus_alpha * xl_roll;
	if (r)
		*r = roll;
	// pitch and yaw not supported yet (since actually not necessary)
}
