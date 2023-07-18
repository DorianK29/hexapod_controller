#pragma once

#include <webots/Motor.hpp>

#define DEG_TO_RAD M_PI / 180
#define RAD_TO_DEG 180 / M_PI

struct servo
{
    // a motor variable for a servo
    webots::Motor *motor;
    // calculated angle
    float angle;
    const float motorSpeed = 10; // 0.37

    // UNUSED:
    // the minimum and maximum values a servo is allowed to go to
    // const float min = -1.5, max = 1.5;

    // convert a servo's angle from degrees to radians
    void degreesToRadians()
    {
        angle = angle * DEG_TO_RAD;
    }
    // convert a servo's angle from radians to degrees
    void radiansToDegrees()
    {
        angle = angle * RAD_TO_DEG;
    }

    // debug
    float angleFix;
};