#include "plane.h"
#include "I2Cdev.h"

Sensors& Sensors::instance()
{
    static Sensors inst;
    return inst;
}

Sensors::Sensors()
{
    _available = false;
}

void Sensors::setup()
{
    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();
    pinMode(2, INPUT);
    auto dmpStatus = mpu.dmpInitialize();

    if (dmpStatus == 0)
    {
        mpu.setDMPEnabled(true);
        _available = true;
    }
}

void Sensors::update()
{
    const float deg2rad = (float)M_PI/180.0f;
    static uint8_t fifoBuffer[64];

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) //DMP에서 수신하면 값을 갱신 (자체적으로 overflow 해결해줌)
    {
        Quaternion q;
        VectorFloat g;
        VectorInt16 int_gyro;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&g, &q);
        mpu.dmpGetYawPitchRoll(reinterpret_cast<float*>(&ypr_angle), &q,&g);
        mpu.dmpGetGyro(&int_gyro, fifoBuffer);
        
        gyro.x = int_gyro.x*deg2rad;
        gyro.y = int_gyro.y*deg2rad;
        gyro.z = int_gyro.z*deg2rad;
    }    
}

bool Sensors::avilable() const
{
    return _available;
}

vec3 Sensors::getGyro()
{
    return gyro;
}

vec3 Sensors::getAngle()
{
    return ypr_angle;
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
    w_ail2.attach(2, mic_min, mic_max);
    w_elev.attach(3, mic_min, mic_max);
    w_rud.attach(4, mic_min, mic_max);
#else
    w_ail1.attach(1);
    w_ail2.attach(2);
    w_elev.attach(3);
    w_rud.attach(4);

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
