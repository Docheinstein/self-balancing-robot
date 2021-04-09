#include "madgwick_filter.h"
#include "math.h"
#include "verbose.h"
#include "std.h"

#define VERBOSE_FMT(fmt) "{MADGWICK_FILTER} " fmt

static float beta;
static float sample_rate;
static float sample_time;
static float q0, q1, q2, q3;
static float isqrt(float x);

void MadgwickFilter_Init(MadgwickFilter_Config config)
{
	verboseln(
		"Initialized" SERIAL_ENDL
		"- beta: %f" SERIAL_ENDL
		"- sample_rate: %.2f",
		config.beta,
		config.sample_rate
	);

	beta = config.beta;
	sample_rate = config.sample_rate;
	sample_time = 1 / config.sample_rate;
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
}

void MadgwickFilter_Compute(dim3_f xl, dim3_f g, float *roll, float *pitch, float *yaw)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	float gx = DEG_TO_RAD(g.x);
	float gy = DEG_TO_RAD(g.y);
	float gz = DEG_TO_RAD(g.z);
	float ax = xl.x;
	float ay = xl.y;
	float az = xl.z;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = isqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = isqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sample_rate);
	q1 += qDot2 * (1.0f / sample_rate);
	q2 += qDot3 * (1.0f / sample_rate);
	q3 += qDot4 * (1.0f / sample_rate);

	// Normalise quaternion
	recipNorm = isqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	if (roll)
		*roll = RAD_TO_DEG(atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2));
	if (pitch)
		*pitch = RAD_TO_DEG(asinf(-2.0f * (q1 * q3 - q0 * q2)));
	if (yaw)
		*yaw = RAD_TO_DEG(atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3));
}

// http://en.wikipedia.org/wiki/Fast_inverse_square_root
static float isqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
//	i = 0x5f3759df - (i>>1); // original
	i = 0x5f375a86 - (i>>1); // proved to be better
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
