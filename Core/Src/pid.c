#include "pid.h"
#include "verbose.h"
#include "std.h"


#define VERBOSE_FMT(fmt) "{PID} " fmt
static PID_Config config;
static float sample_time;
static float Kp, Ki_, Kd_;
static uint32_t last_compute_time;

void PID_Init(PID_Config c)
{
	config = c;

	sample_time = 1 / config.sample_rate;

	float sign = ((config.direction == PID_DIRECTION_DIRECT) ? 1 : -1);
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

float PID_Compute(uint32_t time, float input)
{
	return PID_ComputeCustomSetpoint(time, input, config.setpoint);
}

float PID_ComputeCustomSetpoint(uint32_t time, float input, float setpoint)
{
	static float e_sum = 0;
	static float e_last = 0;

	float e = setpoint - input;

	e_sum += e;

	float p = Kp * e;
	float i = Ki_ * e_sum;
	float d = Kd_ * (e - e_last) * (last_compute_time > 0);
	float pid = p + i + d;

	if (config.limit_output)
		pid = rangef(pid, config.output_limits.min, config.output_limits.max);

	e_last = e;
	last_compute_time = time;

	return pid;
}

