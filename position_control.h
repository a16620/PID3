#pragma once
#include "pid.h"
#include "plane.h"

//3축 PID제어를 위한 클래스
class PositionControl {
    const int pid_max = Actuators::range()/2;
    const float pid_i_max = 70;

    DualLoopPID pid_pitch, pid_roll, pid_yaw; //각 축에 PID

    Sensors& sensors;
    Actuators& actuators;

public:
    PositionControl();
    void process(float pitch, float roll, float yaw); //각도을 받아 각각 PID적용
};

