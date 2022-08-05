#pragma once
#include "pid.h"
#include "plane.h"

class PositionControl {
    const int pid_max = Actuators::range()/2;
    const float pid_i_max = 70;

    DualLoopPID pid_pitch, pid_roll, pid_yaw;

    Sensors& sensors;
    Actuators& actuators;

public:
    PositionControl();
    void process(float pitch, float roll, float yaw);
};

