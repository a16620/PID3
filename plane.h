#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "vec_math.h"

#define YAW z
#define PITCH y
#define ROLL x

//센서를 담당하는 클래스. 싱글톤
class Sensors {
    MPU6050 mpu;

    bool _available;
    Quat quat_angle;
    vec3 gyro, ypr_angle;

    Sensors();
public:
    Sensors(const Sensors&) = delete;
	Sensors& operator=(const Sensors&) = delete;

    static Sensors& instance();

    void setup(); //센서 사용을 위한 준비 함수
    void update(); //센서값 갱신
    bool avilable() const;

    vec3 getGyro(); //현재 각속도
    vec3 getAngle(); //현재 각도
};

//서보, 프로펠러 제어를 위한 클래스. 싱글톤
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

    void setup(); //핀등을 설정

	void setAiler1(const int& value);
	void setAiler2(const int& value);

	void setRudder(const int& value);
	
	void setElev(const int& value);

};