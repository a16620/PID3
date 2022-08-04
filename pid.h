#pragma once

constexpr float MATH_PI = 3.141592653589793f;

class DualLoopPID
{
private:
	float rP, rI, rD, pP;
	float last_error, last_deriv, integ_acc;

	const float lp_filter = 1 / (2 * MATH_PI * 60); //(2 * MATH_PI * frequency)
	float integ_max;
	int output_max;

	inline float position_p(const float& target, const float& position);

	inline float rate_p(const float& error);
	inline float rate_i(const float& error, const float& dt);
	inline float rate_d(const float& error, const float& dt);

public:
	DualLoopPID(float pp, float p, float i, float d, int max_ouput, float max_integ);
	DualLoopPID(float pp, float p, float i, float d);

	int pid(const float& target, const float& position, const float& speed, const float& dt);
	void reset();

	void max_output(const int& m);
	void max_integ(const int& m);
};

