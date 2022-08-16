#include "position_control.h"

PositionControl::PositionControl() : sensors(Sensors::instance()), actuators(Actuators::instance()),
                                        pid_pitch(7, 0.5, 0.15, 0.05,pid_max, pid_i_max), pid_roll(7, 0.5, 0.15, 0.05,pid_max,pid_i_max), pid_yaw(7, 0.5, 0.15, 0.05,pid_max,pid_i_max)
{

}

void PositionControl::process(float pitch, float roll, float yaw)
{
    const auto angle = sensors.getAngle(), gyro = sensors.getGyro(); //현재 각도와 각속도 얻음
    const auto dt = DeltaTime::delta();

    //PID 제어값 구하기
    auto pitch_act = pid_pitch.pid(pitch, angle.PITCH, gyro.PITCH, dt)+Actuators::bias(); //bias를 더해 서보에 전달 가능하게 범위를 변경 (- ~ +)->(0 ~ +)
    auto roll_act = pid_roll.pid(roll, angle.ROLL, gyro.ROLL, dt)+Actuators::bias();
    auto yaw_act = pid_yaw.pid(yaw, angle.YAW, gyro.YAW, dt)+Actuators::bias();

    //서보에 신호 전달
    actuators.setElev(pitch_act);
    actuators.setAiler1(roll_act);
    actuators.setAiler2(Actuators::inverse(roll_act));
    actuators.setRudder(yaw_act);
}