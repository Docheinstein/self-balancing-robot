#include "complementary_filter.h"
#include "math.h"
#include "std.h"
#include "verbose.h"
#include "serial.h"


#define VERBOSE_FMT(fmt) "{FILTER} " fmt

// For debug purpose
#define DISPLAY_RAW_ROLL false

void ComplementaryFilter_Init(
		ComplementaryFilter *filter,
		ComplementaryFilter_Config config)
{
	averboseln(
		"Initialized" SERIAL_ENDL
		"- alpha: %f" SERIAL_ENDL
		"- sample_rate: %.2f",
		config.alpha,
		config.sample_rate
	);

	filter->config = config;
	filter->_.alpha = config.alpha;
	filter->_.one_minus_alpha = 1 - config.alpha;
	filter->_.sample_time = 1 / config.sample_rate;
	filter->roll = filter->pitch = filter->yaw = NAN;

	filter->_.g_roll = NAN;
}

void ComplementaryFilter_Compute(
		ComplementaryFilter *filter,
		dim3_f xl, dim3_f g)
{
	// Actually pitch and yaw are not computed since not necessary
	float xl_roll = RAD_TO_DEG(atan2f(xl.y, xl.z));
	if (isnanf(filter->roll)) {// just first time
		filter->roll = filter->_.g_roll = xl_roll;
	}

	filter->_.g_roll += g.x * filter->_.sample_time;
	filter->roll = filter->_.alpha * (filter->_.g_roll) +
				 filter->_.one_minus_alpha * xl_roll;

#if DISPLAY_RAW_ROLL
	aprintln("xl_roll = %f | g_roll = %f | filter_roll = %f", xl_roll, filter->_.g_roll, filter->roll);
#endif

}
