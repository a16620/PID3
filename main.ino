#include "plane.h"

Sensors& sensors = Sensors::instance();
Actuators& actuators = Actuators::instance();

void setup()
{
	//DeltaTime::init();

    sensors.setup();
    actuators.setup();


    Serial.begin(9600);
    while (!Serial);
}

void loop()
{
    if (!sensors.avilable())
        return;
    
	//DeltaTime::update();
    sensors.update();


    static int cnt = 0;
    if (++cnt == 250) {
        auto angle = sensors.getAngle()*(180.0f/M_PI);
        Serial.print("YAW=");
        Serial.println(angle.x);
        Serial.print("PIT=");
        Serial.println(angle.y);
        Serial.print("ROL=");
        Serial.println(angle.z);

    cnt = 0;
    }

    delay(5);
}
