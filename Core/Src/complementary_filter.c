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
		filter->roll = xl_roll;
#if DISPLAY_RAW_ROLL
		filter->_.g_roll = xl_roll;
#endif
	}

#if DISPLAY_RAW_ROLL
	float filter_roll_before = filter->roll;
#endif


	filter->roll = filter->_.alpha * (filter->roll + g.x * filter->_.sample_time) +
				   filter->_.one_minus_alpha * xl_roll;

#if DISPLAY_RAW_ROLL
	filter->_.g_roll += g.x * filter->_.sample_time;
	aprintln("g.x = % .4f | g_roll = % .4f | xl_roll = % .4f | filter_roll_before % .4f | filter_roll = % .4f",
			g.x, filter->_.g_roll, xl_roll, filter_roll_before, filter->roll);
#endif

}
