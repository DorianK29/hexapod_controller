#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <math.h>

#define joint1 44.75
#define joint2 46.25
#define joint3 65.7
#define PI 3.14159265359

#define DEG_TO_RAD PI / 180
#define RAD_TO_DEG 180 / PI

using namespace webots;
using namespace std;

struct servo
{
    // a motor variable for a servo
    Motor *motor;
    // calculated angle
    float angle;
    // the minimum and maximum values a servo is allowed to go to
    float min = -1.5, max = 1.5;
    // convert a servo's angle from degrees to radians
    void DegToRad()
    {
        angle = angle * DEG_TO_RAD;
    }
    // convert a servo's angle from radians to degrees
    void RadToDeg()
    {
        angle = angle * RAD_TO_DEG;
    }
};

// a 3d vector
struct vector3
{
    // displacements on a given axis
    float x, y, z;
    // returns the length of the vector
    float length()
    {
        float l = sqrt(x * x + y * y + z * z);
        return l;
    }
};

struct leg
{
    // first servo, connecting the body and the first joint of a leg
    servo servo1;
    // second servo, connecting the first joint of a leg and the second joint of a leg
    servo servo2;
    // third servo, connecting the second joint of a leg and the third joint of a leg
    servo servo3;
    // from the center of the body to the point of contact with the ground
    vector3 D;
    // from the center point of rotation of servo 1 to the point of contact with the ground
    vector3 l;
    // from the center point of rotation of servo 2 to the point of contact with the ground
    vector3 L;
    // from the center of the body to the center point of rotation of servo 1
    vector3 s1;
    // from the center of the body to the center point of rotation of servo 2
    vector3 s2;
    // angle
    float angle;
    // convert all servo angles from radians to degrees
    void servoRadToDeg()
    {
        servo1.RadToDeg();
        servo2.RadToDeg();
        servo3.RadToDeg();
    }
    // convert all servo angles from degrees to radians
    void servoDegToRad()
    {
        servo1.DegToRad();
        servo2.DegToRad();
        servo3.DegToRad();
    }
    // return a servo variable for the given servo number
    // 1: servo1
    // 2: servo2
    // 3: servo3
    servo &servoSwitch(int number)
    {
        switch (number)
        {
        case 1:
            return servo1;
            break;
        case 2:
            return servo2;
            break;
        case 3:
            return servo3;
            break;
        }
        return servo1; // to stop compiler error
    }
};
