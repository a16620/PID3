#include "sensor_filter.h"
#include <Arduino.h>
#include <math.h>

Kalman::Kalman() : x{0,0}, p{{0,0},{0,0}}
{

}

float Kalman::operator()(float u, float y, float dt)
{
    x[0] += (u-x[1])*dt;
    p[0][1] -= p[1][1]*dt;
    p[0][0] -= (p[1][0]+p[0][1]+q1)*dt;
    p[1][0] -= p[1][1]*dt;
    p[1][1] += q2*dt;

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

void Madgwick::updateAHRS(vec3 g, vec3 a, vec3 m) {
	Quat s, qDot;
	float hx, hy;
	float _2qwmx, _2qwmy, _2qwmz, _2qxmx, _2bx, _2bz, _4bx, _4bz, _2qw, _2qx, _2qy, _2qz, _2qwqy, _2qyqz, qw_2, qwqx, qwqy, qwqz, qx_2, qxqy, qxqz, qy_2, qyqz, qz_2;

	if((m.x == 0.0f) && (m.y == 0.0f) && (m.z == 0.0f)) {
		updateIMU(g, a);
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
		_2qwmx = 2.0f * q.w * m.x;
		_2qwmy = 2.0f * q.w * m.y;
		_2qwmz = 2.0f * q.w * m.z;
		_2qxmx = 2.0f * q.x * m.x;
		_2qw = 2.0f * q.w;
		_2qx = 2.0f * q.x;
		_2qy = 2.0f * q.y;
		_2qz = 2.0f * q.z;
		_2qwqy = 2.0f * q.w * q.y;
		_2qyqz = 2.0f * q.y * q.z;
		qw_2 = q.w * q.w;
		qwqx = q.w * q.x;
		qwqy = q.w * q.y;
		qwqz = q.w * q.z;
		qx_2 = q.x * q.x;
		qxqy = q.x * q.y;
		qxqz = q.x * q.z;
		qy_2 = q.y * q.y;
		qyqz = q.y * q.z;
		qz_2 = q.z * q.z;

		// Reference direction of Earth's magnetic field
		hx = m.x * qw_2 - _2qwmy * q.z + _2qwmz * q.y + m.x * qx_2 + _2qx * m.y * q.y + _2qx * m.z * q.z - m.x * qy_2 - m.x * qz_2;
		hy = _2qwmx * q.z + m.y * qw_2 - _2qwmz * q.x + _2qxmx * q.y - m.y * qx_2 + m.y * qy_2 + _2qy * m.z * q.z - m.y * qz_2;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2qwmx * q.y + _2qwmy * q.x + m.z * qw_2 + _2qxmx * q.z - m.z * qx_2 + _2qy * m.y * q.z - m.z * qy_2 + m.z * qz_2;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s.w = -_2qy * (2.0f * qxqz - _2qwqy - a.x) + _2qx * (2.0f * qwqx + _2qyqz - a.y) - _2bz * q.y * (_2bx * (0.5f - qy_2 - qz_2) + _2bz * (qxqz - qwqy) - m.x) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (qxqy - qwqz) + _2bz * (qwqx + qyqz) - m.y) + _2bx * q.y * (_2bx * (qwqy + qxqz) + _2bz * (0.5f - qx_2 - qy_2) - m.z);
		s.x = _2qz * (2.0f * qxqz - _2qwqy - a.x) + _2qw * (2.0f * qwqx + _2qyqz - a.y) - 4.0f * q.x * (1 - 2.0f * qx_2 - 2.0f * qy_2 - a.z) + _2bz * q.z * (_2bx * (0.5f - qy_2 - qz_2) + _2bz * (qxqz - qwqy) - m.x) + (_2bx * q.y + _2bz * q.w) * (_2bx * (qxqy - qwqz) + _2bz * (qwqx + qyqz) - m.y) + (_2bx * q.z - _4bz * q.x) * (_2bx * (qwqy + qxqz) + _2bz * (0.5f - qx_2 - qy_2) - m.z);
		s.y = -_2qw * (2.0f * qxqz - _2qwqy - a.x) + _2qz * (2.0f * qwqx + _2qyqz - a.y) - 4.0f * q.y * (1 - 2.0f * qx_2 - 2.0f * qy_2 - a.z) + (-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5f - qy_2 - qz_2) + _2bz * (qxqz - qwqy) - m.x) + (_2bx * q.x + _2bz * q.z) * (_2bx * (qxqy - qwqz) + _2bz * (qwqx + qyqz) - m.y) + (_2bx * q.w - _4bz * q.y) * (_2bx * (qwqy + qxqz) + _2bz * (0.5f - qx_2 - qy_2) - m.z);
		s.z = _2qx * (2.0f * qxqz - _2qwqy - a.x) + _2qy * (2.0f * qwqx + _2qyqz - a.y) + (-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5f - qy_2 - qz_2) + _2bz * (qxqz - qwqy) - m.x) + (-_2bx * q.w + _2bz * q.y) * (_2bx * (qxqy - qwqz) + _2bz * (qwqx + qyqz) - m.y) + _2bx * q.x * (_2bx * (qwqy + qxqz) + _2bz * (0.5f - qx_2 - qy_2) - m.z);
		
        s.normalize();

		// Apply feedback step
		qDot = qDot-s*beta;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q = q + qDot * (1.0f / sampleFreq);

	// Normalise quaternion
	q.normalize();
}

void Madgwick::updateIMU(vec3 g, vec3 a) {
	Quat s, qDot;
	float _2qw, _2qx, _2qy, _2qz, _4qw, _4qx, _4qy, _8qx, _8qy, qw_2, qx_2, qy_2, qz_2;


	// Rate of change of quaternion from gyroscope
	qDot = q*g*0.5f;


	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!a.isZero()) {

		// Normalise accelerometer measurement
		a.normalize();  

		// Auxiliary variables to avoid repeated arithmetic
		_2qw = 2.0f * q.w;
		_2qx = 2.0f * q.x;
		_2qy = 2.0f * q.y;
		_2qz = 2.0f * q.z;
		_4qw = 4.0f * q.w;
		_4qx = 4.0f * q.x;
		_4qy = 4.0f * q.y;
		_8qx = 8.0f * q.x;
		_8qy = 8.0f * q.y;
		qw_2 = q.w * q.w;
		qx_2 = q.x * q.x;
		qy_2 = q.y * q.y;
		qz_2 = q.z * q.z;

		// Gradient decent algorithm corrective step
		s.w = _4qw * qy_2 + _2qy * a.x + _4qw * qx_2 - _2qx * a.y;
		s.x = _4qx * qz_2 - _2qz * a.x + 4.0f * qw_2 * q.x - _2qw * a.y - _4qx + _8qx * qx_2 + _8qx * qy_2 + _4qx * a.z;
		s.y = 4.0f * qw_2 * q.y + _2qw * a.x + _4qy * qz_2 - _2qz * a.y - _4qy + _8qy * qx_2 + _8qy * qy_2 + _4qy * a.z;
		s.z = 4.0f * qx_2 * q.z - _2qx * a.x + 4.0f * qy_2 * q.z - _2qy * a.y;

		s.normalize();

		// Apply feedback step
		qDot = qDot-s*beta;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q = q + qDot * (1.0f / sampleFreq);


	// Normalise quaternion
	q.normalize();
}

vec3 Madgwick::getEuler() const
{
	vec3 euler;

	float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    euler.x = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (abs(sinp) >= 1)
        euler.y = copysignf(M_PI / 2, sinp);
    else
        euler.y = asinf(sinp);

    
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    euler.z = atan2f(siny_cosp, cosy_cosp);

	return euler;
}
