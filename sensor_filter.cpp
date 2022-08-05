#include "sensor_filter.h"

Kalman::Kalman() : x({}), p({{},{}})
{

}

float Kalman::operator()(float u, float y, float dt)
{
    x[0] += (u-x[1])*dt;
    p[0][1] -= p[1][1]*dt;
    p[0][0] -= (p[1][0]+p[0][1]+q.x)*dt;
    p[1][0] -= p[1][1]*dt;
    p[1][1] += q.y*dt;

    float s = p[0][0]+r1, k1 = p[0][0]/s, k2 = p[1][0]/s, e = y-x[0];
    x[0] += k1*e;
    x[1] += k2*e;

    p[0][0] *= (1-k1);
    p[0][1] *= (1-k1);
    p[1][0] -= k2*p[0][0];
    p[1][1] -= k2*p[0][1];

    return x[0];
}

float Kalman::get() const
{
    return x[0];
}

Madgwick::Madgwick()
{
    q = Quat(1.0f, {});
}

void Madgwick::UpdateAHRS(vec3 g, vec3 a, vec3 m) {
	Quat s, qDot;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	if((m.x == 0.0f) && (m.y == 0.0f) && (m.z == 0.0f)) {
		UpdateIMU(g, a);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot = q*g*0.5f;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!a.isZero()) {

		// Normalise accelerometer measurement
		a.normalize();

		// Normalise magnetometer measurement
		m.normalize();

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q.w * m.x;
		_2q0my = 2.0f * q.w * m.y;
		_2q0mz = 2.0f * q.w * m.z;
		_2q1mx = 2.0f * q.x * m.x;
		_2q0 = 2.0f * q.w;
		_2q1 = 2.0f * q.x;
		_2q2 = 2.0f * q.y;
		_2q3 = 2.0f * q.z;
		_2q0q2 = 2.0f * q.w * q.y;
		_2q2q3 = 2.0f * q.y * q.z;
		q0q0 = q.w * q.w;
		q0q1 = q.w * q.x;
		q0q2 = q.w * q.y;
		q0q3 = q.w * q.z;
		q1q1 = q.x * q.x;
		q1q2 = q.x * q.y;
		q1q3 = q.x * q.z;
		q2q2 = q.y * q.y;
		q2q3 = q.y * q.z;
		q3q3 = q.z * q.z;

		// Reference direction of Earth's magnetic field
		hx = m.x * q0q0 - _2q0my * q.z + _2q0mz * q.y + m.x * q1q1 + _2q1 * m.y * q.y + _2q1 * m.z * q.z - m.x * q2q2 - m.x * q3q3;
		hy = _2q0mx * q.z + m.y * q0q0 - _2q0mz * q.x + _2q1mx * q.y - m.y * q1q1 + m.y * q2q2 + _2q2 * m.z * q.z - m.y * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q.y + _2q0my * q.x + m.z * q0q0 + _2q1mx * q.z - m.z * q1q1 + _2q2 * m.y * q.z - m.z * q2q2 + m.z * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s.w = -_2q2 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q1 * (2.0f * q0q1 + _2q2q3 - a.y) - _2bz * q.y * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + _2bx * q.y * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
		s.x = _2q3 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q0 * (2.0f * q0q1 + _2q2q3 - a.y) - 4.0f * q.x * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a.z) + _2bz * q.z * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (_2bx * q.y + _2bz * q.w) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + (_2bx * q.z - _4bz * q.x) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
		s.y = -_2q0 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q3 * (2.0f * q0q1 + _2q2q3 - a.y) - 4.0f * q.y * (1 - 2.0f * q1q1 - 2.0f * q2q2 - a.z) + (-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (_2bx * q.x + _2bz * q.z) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + (_2bx * q.w - _4bz * q.y) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
		s.z = _2q1 * (2.0f * q1q3 - _2q0q2 - a.x) + _2q2 * (2.0f * q0q1 + _2q2q3 - a.y) + (-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m.x) + (-_2bx * q.w + _2bz * q.y) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m.y) + _2bx * q.x * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - m.z);
		
        s.normalize();

		// Apply feedback step
		qDot = qDot-s*beta;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q = q + qDot * (1.0f / sampleFreq);

	// Normalise quaternion
	q.normalize();
}

void Madgwick::UpdateIMU(vec3 g, vec3 a) {
	Quat s, qDot;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot = q*g*0.5f;


	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!a.isZero()) {

		// Normalise accelerometer measurement
		a.normalize();  

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q.w;
		_2q1 = 2.0f * q.x;
		_2q2 = 2.0f * q.y;
		_2q3 = 2.0f * q.z;
		_4q0 = 4.0f * q.w;
		_4q1 = 4.0f * q.x;
		_4q2 = 4.0f * q.y;
		_8q1 = 8.0f * q.x;
		_8q2 = 8.0f * q.y;
		q0q0 = q.w * q.w;
		q1q1 = q.x * q.x;
		q2q2 = q.y * q.y;
		q3q3 = q.z * q.z;

		// Gradient decent algorithm corrective step
		s.w = _4q0 * q2q2 + _2q2 * a.x + _4q0 * q1q1 - _2q1 * a.y;
		s.x = _4q1 * q3q3 - _2q3 * a.x + 4.0f * q0q0 * q.x - _2q0 * a.y - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * a.z;
		s.y = 4.0f * q0q0 * q.y + _2q0 * a.x + _4q2 * q3q3 - _2q3 * a.y - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * a.z;
		s.z = 4.0f * q1q1 * q.z - _2q1 * a.x + 4.0f * q2q2 * q.z - _2q2 * a.y;

		s.normalize();

		// Apply feedback step
		qDot = qDot-s*beta;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q = q + qDot * (1.0f / sampleFreq);


	// Normalise quaternion
	q.normalize();
    
}

