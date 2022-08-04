#pragma once
#include <Arduino.h>

class DeltaTime {
    static long prevTime;
    static float delta_;
public:
    static void init(void)
    {
        prevTime = micros();
        delta_ = 0.0f;
    }

    static void update(void)
    {
        auto now = micros();
        delta_ = static_cast<float>(now-prevTime) / 1000000.0f;
        prevTime = now;
    }

    static float delta()
    {
        return delta_;
    }
};