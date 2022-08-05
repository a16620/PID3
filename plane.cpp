#include "plane.h"

Sensors& Sensors::instance()
{
    static Sensors inst;
    return inst;
}

Sensors::Sensors() : accel_sqrt(0.0f), mag_direction(0.0f)
{
}

void Sensors::setup()
{
    Wire.begin();
    mpu.setWire(&Wire);

    bmp.begin();
    mpu.beginAccel();
    mpu.beginGyro();
    mpu.beginMag();
}

void Sensors::update()
{
    if (mpu.accelUpdate() == 0)
    {
        accel.x = mpu.accelX();
        accel.y = mpu.accelY();
        accel.z = mpu.accelZ();

        accel_sqrt = mpu.accelSqrt();
    }

    if (mpu.gyroUpdate() == 0)
    {
        gyro.x = mpu.gyroX();
        gyro.y = mpu.gyroY();
        gyro.z = mpu.gyroZ();
    }

    if (mpu.magUpdate() == 0)
    {
        mag.x = mpu.magX();
        mag.y = mpu.magY();
        mag.z = mpu.magZ();

        mag_direction = mpu.magHorizDirection();
    }
}

vec3 Sensors::getGyro()
{
    return gyro;
}

vec3 Sensors::getAccel()
{
    return accel;
}

vec3 Sensors::getAngle()
{
    return vec3();
}

inline int Actuators::map(const int x, const int x_min, const int x_max)
{
    return ::map(x, x_min, x_max, act_min, act_max);
}

inline constexpr int Actuators::range()
{
    return act_max-act_min;
}

inline constexpr int Actuators::bias()
{
    return (act_max+act_min)/2;
}

inline constexpr int Actuators::inverse(int x)
{
    return (act_max+act_min) - x;
}

Actuators& Actuators::instance()
{
    static Actuators inst;
    return inst;
}

Actuators::Actuators()
{
}

void Actuators::setup()
{

}

void Actuators::setAiler1(const int& value)
{

}

void Actuators::setAiler2(const int& value)
{
    
}

void Actuators::setElev(const int& value)
{
    
}

void Actuators::setRudder(const int& value)
{
    
}
