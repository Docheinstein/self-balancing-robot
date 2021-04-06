#include "complementary_filter.h"
#include "math.h"
#include "std.h"
#include "verbose.h"
#include "serial.h"

#define VERBOSE_FMT(fmt) "{FILTER} " fmt

static float alpha;
static float one_minus_alpha;
static float sample_time_s;
static float angle;

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
	sample_time_s = 1 / config.sample_rate;
	angle = NAN;
}

float ComplementaryFilter_Compute(dim3_f xl, dim3_f g)
{
	static float g_angle; // debug

	float xl_angle = RAD_TO_DEG(atan2f(xl.y, xl.z));
	if (isnanf(angle)) { // just first time
		angle = xl_angle;
		g_angle = xl_angle; // debug
	}
	g_angle += g.x * sample_time_s; // debug
	angle = alpha * (angle + g.x * sample_time_s) + one_minus_alpha * xl_angle;

	verboseln("XL: %f°, G: %f°, FILT: %f°", xl_angle, g_angle, angle);
	return angle;
}
