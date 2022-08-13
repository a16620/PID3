#include "plane.h"

Sensors& Sensors::instance()
{
    static Sensors inst;
    return inst;
}

Sensors::Sensors()
{
}

void Sensors::setup()
{
    Wire.begin();
    mpu.initMPU6050();
}

void Sensors::update()
{
    static int16_t sensor_cnt[3];
    const float deg2rad = (float)M_PI/180.0f;

    if (mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01)
    {
        mpu.readAccelData(sensor_cnt);
        auto accel_res = mpu.getAres();

        accel.x = sensor_cnt[0]*accel_res;
        accel.y = sensor_cnt[1]*accel_res;
        accel.z = sensor_cnt[2]*accel_res;

        mpu.readGyroData(sensor_cnt);
        auto gyro_res = mpu.getGres()*deg2rad;

        gyro.x = sensor_cnt[0]*gyro_res;
        gyro.y = sensor_cnt[1]*gyro_res;
        gyro.z = sensor_cnt[2]*gyro_res;
    }

    mad.updateIMU(gyro, accel);
    filtered_angle = mad.getEuler();
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
    return filtered_angle;
}

#ifdef USE_MICROSEC_WRITE
int Actuators::mic_map(const int x)
{
    return ::map(x, act_min, act_max, mic_min, mic_max);
}
#endif

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
#ifdef USE_MICROSEC_WRITE
    w_ail1.attach(1, mic_min, mic_max);
    w_ail2.attach(1, mic_min, mic_max);
    w_elev.attach(1, mic_min, mic_max);
    w_rud.attach(1, mic_min, mic_max);
#else
    w_ail1.attach(1);
    w_ail2.attach(1);
    w_elev.attach(1);
    w_rud.attach(1);

#endif
}

void Actuators::setAiler1(const int& value)
{
#ifdef USE_MICROSEC_WRITE
    w_ail1.writeMicroseconds(Actuators::mic_map(value));
#else
    w_ail1.write(value);
#endif
}

void Actuators::setAiler2(const int& value)
{
#ifdef USE_MICROSEC_WRITE
    w_ail2.writeMicroseconds(Actuators::mic_map(value));
#else
    w_ail2.write(value);
#endif
}

void Actuators::setElev(const int& value)
{
#ifdef USE_MICROSEC_WRITE
    w_elev.writeMicroseconds(Actuators::mic_map(value));
#else
    w_elev.write(value);
#endif
}

void Actuators::setRudder(const int& value)
{
#ifdef USE_MICROSEC_WRITE
    w_rud.writeMicroseconds(Actuators::mic_map(value));
#else
    w_rud.write(value);
#endif
}
