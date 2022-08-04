#pragma once
#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h>

#define YAW z
#define PITCH y
#define ROLL x

struct vec3f {
    float x, y, z;
};

class Sensors {
    Adafruit_BMP280 bmp;
    MPU9250_asukiaaa mpu;
    
    vec3f accel, gyro, mag;
    float accel_sqrt, mag_direction;

    Sensors();
public:
    Sensors(const Sensors&) = delete;
	Sensors& operator=(const Sensors&) = delete;

    static Sensors& instance()
    {
        static Sensors inst;
        return inst;
    }

    void setup();
    void update();

    vec3f getGyro();
    vec3f getAccel();
    vec3f getAngle();
};

class Actuators {
    static constexpr int act_min = 0, act_max = 255;

    Actuators();
public:
    Actuators(const Actuators&) = delete;
	Actuators& operator=(const Actuators&) = delete;

    static Actuators& instance()
    {
        static Actuators inst;
        return inst;
    }

    inline static int map(const int x, const int x_min, const int x_max)
    {
        return ::map(x, x_min, x_max, act_min, act_max);
    }

    inline static constexpr int range()
    {
        return act_max-act_min;
    }

    inline static constexpr int bias()
    {
        return (act_max+act_min)/2;
    }

    inline static constexpr int inverse(int x)
    {
        return act_max - x + act_min;
    }

    void setup();

	void setAiler1(const int& value);
	void setAiler2(const int& value);

	void setRudder(const int& value);
	
	void setElev(const int& value);

};