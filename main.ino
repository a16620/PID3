#include "plane.h"

Sensors& sensors = Sensors::instance();
Actuators& actuators = Actuators::instance();

void setup()
{
    sensors.setup(); //센서 준비
    actuators.setup(); //서보 준비


    Serial.begin(9600);
    while (!Serial);
}

void loop()
{
    if (!sensors.avilable()) //센서에 오류가 있으면 종료
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
