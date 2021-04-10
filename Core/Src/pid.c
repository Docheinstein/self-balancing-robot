#include "pid.h"
#include "verbose.h"
#include "std.h"


#define VERBOSE_FMT(fmt) "{PID} " fmt
static PID_Config config;
static float sample_time;
static float Kp, Ki_, Kd_;

void PID_Init(PID_Config c)
{
	config = c;

	sample_time = 1 / config.sample_rate;

	float sign = ((config.direction == PID_DIRECTION_DIRECT) ? 1.0f : -1.0f);
	Kp =  sign * config.Kp;
	Ki_ = sign * config.Ki * sample_time;
	Kd_ = sign * config.Kd / sample_time;

	verboseln(
		"Initialized" SERIAL_ENDL
		"- Kp: %f" SERIAL_ENDL
		"- Ki: %f" SERIAL_ENDL
		"- Kd: %f" SERIAL_ENDL
		"- setpoint: %f" SERIAL_ENDL
		"- sample_rate: %.2f" SERIAL_ENDL
		"- output_limit: [%.2f, %.2f]",
		config.Kp,
		config.Ki,
		config.Kd,
		config.setpoint,
		config.sample_rate,
		config.output_limits.min, config.output_limits.max
	);
}

float PID_Compute(float input)
{
	return PID_ComputeCustomSetpoint(input, config.setpoint);
}

float PID_ComputeCustomSetpoint(float input, float setpoint)
{
	static float e_sum = 0;
	static float e_last = 0;

	float e = setpoint - input;

	e_sum += e;

	float p = Kp * e;
	float i = Ki_ * e_sum;
	float d = Kd_ * (e - e_last);
	float pid = p + i + d;

	e_last = e;

	if (config.limit_output)
		pid = rangef(pid, config.output_limits.min, config.output_limits.max);

	return pid;
}

