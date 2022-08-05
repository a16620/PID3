#pragma once
#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h>
#include "vec_math.h"
#include "sensor_filter.h"

#define YAW z
#define PITCH y
#define ROLL x

class Sensors {
    Adafruit_BMP280 bmp;
    MPU9250_asukiaaa mpu;
    
    vec3 accel, gyro, mag, filtered_angle;

    Madgwick mad;

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

    static int map(const int x, const int x_min, const int x_max);

    static constexpr int range() {
        return act_max-act_min;
    }

    static constexpr int bias() {
        return (act_max+act_min)/2;
    }

    static constexpr int inverse(int x) {
        return (act_max+act_min) - x;
    }

    void setup();

	void setAiler1(const int& value);
	void setAiler2(const int& value);

	void setRudder(const int& value);
	
	void setElev(const int& value);

};