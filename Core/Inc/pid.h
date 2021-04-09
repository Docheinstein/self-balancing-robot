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

void PID_Init(PID_Config config);
float PID_Compute(uint32_t time, float input);
float PID_ComputeCustomSetpoint(uint32_t time, float input, float setpoint);

#endif /* PID_H */
