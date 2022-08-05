#include "position_control.h"

PositionControl ctrl;
vec3 target;

void setup()
{
	DeltaTime::init();

    auto& sensors = Sensors::instance();
    sensors.setup();
    Actuators::instance().setup();

    target = sensors.getAngle();
}

void loop()
{
	DeltaTime::update();
    Sensors::instance().update();

    ctrl.process(target.PITCH, target.ROLL, target.YAW);
}
