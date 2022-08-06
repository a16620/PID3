#include "position_control.h"

PositionControl::PositionControl() : sensors(Sensors::instance()), actuators(Actuators::instance()),
                                        pid_pitch(0.7, 0.5, 0.15, 0.05,pid_max, pid_i_max), pid_roll(0.7, 0.5, 0.15, 0.05,pid_max,pid_i_max), pid_yaw(0.7, 0.5, 0.15, 0.05,pid_max,pid_i_max)
{

}

void PositionControl::process(float pitch, float roll, float yaw)
{
    const auto angle = sensors.getAngle(), gyro = sensors.getGyro();
    const auto dt = DeltaTime::delta();

    auto pitch_act = pid_pitch.pid(pitch, angle.PITCH, gyro.PITCH, dt)+Actuators::bias();
    auto roll_act = pid_pitch.pid(roll, angle.ROLL, gyro.ROLL, dt)+Actuators::bias();
    auto yaw_act = pid_pitch.pid(yaw, angle.YAW, gyro.YAW, dt)+Actuators::bias();

    actuators.setElev(pitch_act);
    actuators.setAiler1(roll_act);
    actuators.setAiler2(Actuators::inverse(roll_act));
    actuators.setRudder(yaw_act);
}