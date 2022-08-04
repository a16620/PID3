#include "position_control.h"

PositionControl ctrl;
vec3f target;

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
    Sensors::instance().update();
	DeltaTime::update();

    ctrl.process(target.PITCH, target.ROLL, target.YAW);
}
