#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	PID_DIRECTION_DIRECT,
	PID_DIRECTION_REVERSE
} PID_Direction;

typedef struct PID_Config {
	float Kp, Ki, Kd;
	float setpoint;
	float sample_rate; // Hz
	bool limit_output;
	struct {
		float min;
		float max;
	} output_limits;
	PID_Direction direction;

} PID_Config;

typedef struct PID {
	PID_Config config;
	float output;
	struct {
		float sample_time; // s
		float Kp, Ki, Kd;
		float e_sum;
		float e_last;
	} _;
} PID;


void PID_Init(PID *pid, PID_Config config);
void PID_Compute(PID *pid, float input);

#endif /* PID_H */
