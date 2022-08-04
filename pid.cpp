#include "PID.h"
#include <Arduino.h>

inline float DualLoopPID::position_p(const float& target, const float& position)
{
	return (target-position)*pP;
}

inline float DualLoopPID::rate_p(const float& error)
{
	return error*rP;
}

inline float DualLoopPID::rate_i(const float& error, const float& dt)
{
	if (rI != 0 && dt != 0) {
		integ_acc += error * rI * dt;

		if (integ_acc > integ_max) {
			integ_acc = integ_max;
		}
		else if (integ_acc < -integ_max) {
			integ_acc = -integ_max;
		}
	}
	return integ_acc;
}

inline float DualLoopPID::rate_d(const float& error, const float& dt)
{
	if (rD != 0 && dt != 0) {
		const float d_value = (error - last_error) / dt;

		auto deriv = last_deriv + (dt / (lp_filter + dt)) * (d_value - last_deriv);

		last_error = error;
		last_deriv = deriv;

		return deriv * rD;
	}

	return 0.0f;
}

DualLoopPID::DualLoopPID(float pp, float p, float i, float d, int max_ouput, float max_integ)
		: output_max(max_ouput), integ_max(max_integ), last_error(0.0f), last_deriv(0.0f), integ_acc(0.0f)
{
	pP = pp;
	rP = p;
	rI = i;
	rD = d;
}

DualLoopPID::DualLoopPID(float pp, float p, float i, float d) : DualLoopPID(pp, p, i, d, 127, 127)
{
}

int DualLoopPID::pid(const float& target, const float& position, const float& speed, const float& dt)
{
	auto rate_target = position_p(target, position);
	auto rate_error = rate_target - speed;
	
	auto pid = static_cast<decltype(output_max)>(rate_p(rate_error) + rate_i(rate_error, dt) + rate_d(rate_error, dt));
	
	return constrain(pid, -output_max, output_max);
}

void DualLoopPID::reset()
{
	integ_acc = 0.0f;
	last_error = 0.0f;
	last_deriv = 0.0f;
}

void DualLoopPID::max_output(const int& m)
{
	output_max = m;
}

void DualLoopPID::max_integ(const int& m)
{
	integ_max = m;
}