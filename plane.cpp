#include "plane.h"
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h>

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

Actuators::Actuators()
{
    
}