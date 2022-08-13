#pragma once
#include "vec_math.h"

class Kalman {
    const float q1=0.001,q2=0.003,r1=0.03;
    float x[2], p[2][2];
public:
    Kalman();

    float operator()(float u, float y, float dt);
    float get() const;
};

class Madgwick {
    const float beta = 0.1f, sampleFreq = 60;
    Quat q;
    
public:
    Madgwick();
    
    void initQuat(const Quat& _q);

    void updateIMU(vec3 g, vec3 a);
    void updateAHRS(vec3 g, vec3 a, vec3 m);

    Quat getQuat() const;
    vec3 getEuler() const;
};