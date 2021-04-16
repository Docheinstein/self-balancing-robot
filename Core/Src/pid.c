#include "pid.h"
#include "verbose.h"
#include "std.h"


#define VERBOSE_FMT(fmt) "{PID} " fmt


void PID_Init(PID *pid, PID_Config config)
{

	pid->config = config;

	pid->_.sample_time = 1 / config.sample_rate;

	float sign = ((config.direction == PID_DIRECTION_DIRECT) ? 1.0f : -1.0f);

	pid->_.Kp = sign * config.Kp;
	pid->_.Ki = sign * config.Ki * pid->_.sample_time;
	pid->_.Kd = sign * config.Kd / pid->_.sample_time;

	pid->_.e_sum = 0;
	pid->_.e_last = 0;


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

void PID_Compute(PID *pid, float input)
{
	float e = pid->config.setpoint - input;

	pid->_.e_sum += e;

	float p = pid->_.Kp * e;
	float i = pid->_.Ki * pid->_.e_sum;
	float d = pid->_.Kd * (e - pid->_.e_last);

	pid->_.e_last = e;

	pid->output = p + i + d;

	if (pid->config.limit_output)
		pid->output = rangef(pid->output, pid->config.output_limits.min, pid->config.output_limits.max);
}

