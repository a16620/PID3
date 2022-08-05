#pragma once
#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h>
#include "vec_math.h"

#define YAW z
#define PITCH y
#define ROLL x

class Sensors {
    Adafruit_BMP280 bmp;
    MPU9250_asukiaaa mpu;
    
    vec3 accel, gyro, mag;
    float accel_sqrt, mag_direction;

    Sensors();
public:
    Sensors(const Sensors&) = delete;
	Sensors& operator=(const Sensors&) = delete;

    static Sensors& instance();

    void setup();
    void update();

    vec3 getGyro();
    vec3 getAccel();
    vec3 getAngle();
};

class Actuators {
    static constexpr int act_min = 0, act_max = 255;

    Actuators();
public:
    Actuators(const Actuators&) = delete;
	Actuators& operator=(const Actuators&) = delete;

    static Actuators& instance();

    inline static int map(const int x, const int x_min, const int x_max);
    inline static constexpr int range();
    inline static constexpr int bias();
    inline static constexpr int inverse(int x);

    void setup();

	void setAiler1(const int& value);
	void setAiler2(const int& value);

	void setRudder(const int& value);
	
	void setElev(const int& value);

};