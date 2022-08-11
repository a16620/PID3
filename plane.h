#pragma once
#include <Arduino.h>
#include <MPU6050.h>
#include <Servo.h>
#include "vec_math.h"
#include "sensor_filter.h"

#define YAW z
#define PITCH y
#define ROLL x

class Sensors {
    MPU6050lib mpu;
    
    vec3 accel, gyro, filtered_angle;

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
    static constexpr int act_min = 30, act_max = 150;
#ifdef USE_MICROSEC_WRITE
    static constexpr int mic_min = 1000, mic_max = 2000;
    static int mic_map(const int x);
#endif

    Servo w_ail1, w_ail2, w_rud, w_elev;

    Actuators();
public:
    Actuators(const Actuators&) = delete;
	Actuators& operator=(const Actuators&) = delete;

    static Actuators& instance();

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