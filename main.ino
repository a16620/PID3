#include "position_control.h"

DualLoopPID pid_pitch(70, 0.5, 0.0, 0.05, 127, 34);
DualLoopPID pid_roll(7, 0.5, 0.15, 0.05, 127, 34);
DualLoopPID pid_yaw(7, 0.5, 0.15, 0.05, 127, 34);

PositionControl ctrl;
vec3 target;

void setup()
{
	DeltaTime::init();

    auto& sensors = Sensors::instance();
    sensors.setup();
    Actuators::instance().setup();

    target = sensors.getAngle();

    Serial.begin(9600);
    while (!Serial);
}

void loop()
{
	DeltaTime::update();
    Sensors::instance().update();

    //ctrl.process(target.PITCH, target.ROLL, target.YAW);
    
    auto q = Sensors::instance().mad.getQuat();
    //auto angle = Sensors::instance().getAngle()*(180.0f/M_PI);
    //auto gyro = Sensors::instance().getGyro();
    //auto dt = DeltaTime::delta();

    //auto pitch_act = pid_pitch.pid(target.PITCH, angle.PITCH, gyro.PITCH, dt);
    //auto roll_act = pid_roll.pid(target.ROLL, angle.ROLL, gyro.ROLL, dt);
    //auto yaw_act = pid_yaw.pid(target.YAW, angle.YAW, gyro.YAW, dt);

    auto f = Quat::rotate(vec3::forward, q);

    static int cnt = 0;
    if (++cnt == 500) {
    Serial.print('(');

    Serial.print(f.x);
    Serial.print(',');
    Serial.print(f.y);
    Serial.print(',');
    Serial.print(f.z);

    Serial.println(')');

    cnt = 0;
    }

    delay(5);
}
