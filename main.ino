#include "position_control.h"

/*
plane.h
    Actuators: 하드웨어 제어
    Sensors: 자이로 센서 값 읽기

vec_math.h
    -벡터, 회전 관련 함수

pid.h
    -PID제어를 위한 기본 클래스

position_control.h
    -3축에 대한 PID제어

*/

Sensors& sensors = Sensors::instance();
Actuators& actuators = Actuators::instance();

PositionControl pctr;

void setup()
{
    sensors.setup(); //센서 준비
    actuators.setup(); //서보 준비

    DeltaTime::init();

    Serial.begin(9600);
    while (!Serial);
}

void loop()
{
    if (!sensors.avilable()) //센서에 오류가 있으면 종료
        return;
    
	DeltaTime::update();
    sensors.update();

    pctr.process(0, 0, 0);

    static unsigned long timer = 0;
    auto now = millis();
    if (now > timer) {
        auto angle = sensors.getAngle()*(180.0f/M_PI);
        Serial.print("YAW=");
        Serial.println(angle.x);
        Serial.print("PIT=");
        Serial.println(angle.y);
        Serial.print("ROL=");
        Serial.println(angle.z);

        timer = now + 2000; //2초마다 각도 출력
    }
}
