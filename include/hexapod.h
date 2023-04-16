#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <math.h>

#define joint1 (float)44.75
#define joint2 (float)46.25
#define joint3 (float)65.7

#define DEG_TO_RAD M_PI / 180
#define RAD_TO_DEG 180 / M_PI

using namespace webots;
using namespace std;

struct servo
{
    // a motor variable for a servo
    Motor *motor;
    // calculated angle
    float angle;
    // angle difference
    float angleFix;
    // the minimum and maximum values a servo is allowed to go to
    float min = -1.5, max = 1.5;
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

    // overload the + operator to add two vector3 structs together
    vector3 operator+(const vector3 &other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    // overload the - operator to subtract two vector3 structs together
    vector3 operator-(const vector3 &other) const
    {
        return {x - other.x, y - other.y, z - other.z};
    }

    // overload the += operator to increment the current vector
    vector3 operator+=(const vector3 &other) const
    {
        return {x + other.x, y + other.y, z + other.z};
    }

    vector3 operator%(const vector3 &other) const
    {
        return vector3{
            other.x != 0 ? other.x : x,
            other.y != 0 ? other.y : y,
            other.z != 0 ? other.z : z};
    }
};

/**
 * Move by setting the vector contactPointFromCenter to desired
 * location. From that vector calculate everything else needed
 *
 */

struct leg
{
    // first servo, connecting the body and the first joint of a leg
    servo servo1;
    // second servo, connecting the first joint of a leg and the second joint of a leg
    servo servo2;
    // third servo, connecting the second joint of a leg and the third joint of a leg
    servo servo3;
    // from the center of the body to the point of contact with the ground
    vector3 contactPointFromCenter;
    // from the center point of rotation of servo 1 to the point of contact with the ground
    vector3 joint1PivotToContactPoint;
    // from the center point of rotation of servo 2 to the point of contact with the ground
    vector3 joint2PivotToContactPoint;
    // from the center of the body to the center point of rotation of servo 1
    vector3 bodyToJoint1Pivot;
    // from the center of the body to the center point of rotation of servo 2
    vector3 bodyToJoint2Pivot;
    // angle at which the leg joint is with reference to the body center
    float theta_z;

    // same vector as contactPointFromCenter
    // the position of the leg when its standing/resting
    vector3 restingPosition;
    // same vector as contactPointFromCenter
    // the farthest position the leg is allowed to be at
    vector3 limitPosition;
    // how much a leg moves in a single step
    float stepDistance;
    // how far up the leg moves while not in contact with the ground
    float stepHeight;
    // the smallest increment of movement for the leg
    float minimalStepDistance;
    // the angle at which the leg is moving
    float walkingAngle;
    // is leg in contact with ground or not
    bool touchingGround;

    // walkingHeight positive is up, negative is down
    float walkingHeight, walkingWidth;

    // convert all servo angles from radians to degrees
    void radiansToDegrees()
    {
        servo1.radiansToDegrees();
        servo2.radiansToDegrees();
        servo3.radiansToDegrees();
    }
    // convert all servo angles from degrees to radians
    void degreesToRadians()
    {
        servo1.degreesToRadians();
        servo2.degreesToRadians();
        servo3.degreesToRadians();
    }

    void setLegAttachmentPos(float newThetaZ, vector3 newBodyToJoint1Pivot)
    {
        theta_z = newThetaZ;
        bodyToJoint1Pivot = newBodyToJoint1Pivot;
    }

    // return a servo variable for the given servo number
    // 1: servo1
    // 2: servo2
    //  3: servo3
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

    void printValues()
    {
        // print values for vector3 variables
        cout << "contactPointFromCenter - x: " << contactPointFromCenter.x << ", y: " << contactPointFromCenter.y << ", z: " << contactPointFromCenter.z << endl;
        cout << "joint1PivotToContactPoint - x: " << joint1PivotToContactPoint.x << ", y: " << joint1PivotToContactPoint.y << ", z: " << joint1PivotToContactPoint.z << endl;
        cout << "joint2PivotToContactPoint - x: " << joint2PivotToContactPoint.x << ", y: " << joint2PivotToContactPoint.y << ", z: " << joint2PivotToContactPoint.z << endl;
        cout << "bodyToJoint1Pivot - x: " << bodyToJoint1Pivot.x << ", y: " << bodyToJoint1Pivot.y << ", z: " << bodyToJoint1Pivot.z << endl;
        cout << "bodyToJoint2Pivot - x: " << bodyToJoint2Pivot.x << ", y: " << bodyToJoint2Pivot.y << ", z: " << bodyToJoint2Pivot.z << endl;
        cout << "restingPosition - x: " << restingPosition.x << ", y: " << restingPosition.y << ", z: " << restingPosition.z << endl;

        // print values for float variables
        cout << "theta_z - " << theta_z * RAD_TO_DEG << endl;
        cout << "stepDistance - " << stepDistance << endl;
        cout << "minimalStepDistance - " << minimalStepDistance << endl;
        cout << "walkingAngle - " << walkingAngle << endl;
        cout << "walkingHeight - " << walkingHeight << endl;
        cout << "walkingWidth - " << walkingWidth << endl;

        radiansToDegrees();
        // print values for motor variables
        cout << "servo1 - angle: " << servo1.angle << ", angleFix: " << servo1.angleFix * RAD_TO_DEG << endl;
        cout << "servo2 - angle: " << servo2.angle << ", angleFix: " << servo2.angleFix * RAD_TO_DEG << endl;
        cout << "servo3 - angle: " << servo3.angle << ", angleFix: " << servo3.angleFix * RAD_TO_DEG << endl;
        degreesToRadians();
    }

    void updateRestPos()
    {
        restingPosition = vector3{walkingWidth * cos(theta_z), walkingWidth * sin(theta_z), walkingHeight};
    }

    void updateLimitPos()
    {
        limitPosition = restingPosition + vector3{cos(walkingAngle) * stepDistance, sin(walkingAngle) * stepDistance, 0};
    }

    void calculate_angles()
    {
        joint1PivotToContactPoint = contactPointFromCenter - bodyToJoint1Pivot;

        servo1.angle = atan(abs(joint1PivotToContactPoint.y) / abs(joint1PivotToContactPoint.x)) - theta_z + servo1.angleFix;

        bodyToJoint2Pivot = bodyToJoint1Pivot + vector3{joint1 * cos(servo1.angle + theta_z), joint1 * sin(servo1.angle + theta_z), 0};

        joint2PivotToContactPoint = contactPointFromCenter - bodyToJoint2Pivot;

        float P = atan(abs(joint2PivotToContactPoint.z) / (sqrt(joint2PivotToContactPoint.x * joint2PivotToContactPoint.x + joint2PivotToContactPoint.y * joint2PivotToContactPoint.y)));
        float R = asin((abs(joint2PivotToContactPoint.z) - abs(joint1PivotToContactPoint.z)) / joint1);
        servo2.angle = -1 * (acos((pow(joint2, 2) + pow(joint2PivotToContactPoint.length(), 2) - pow(joint3, 2)) / (2 * joint2 * joint2PivotToContactPoint.length())) - (P + R));
        servo3.angle = -1 * (M_PI - acos((pow(joint2, 2) + pow(joint3, 2) - pow(joint2PivotToContactPoint.length(), 2)) / (2 * joint2 * joint3)));
    }

    void write_to_servos()
    {
        servo1.motor->setPosition(servo1.angle);
        servo2.motor->setPosition(servo2.angle);
        servo3.motor->setPosition(servo3.angle);
    }

    bool checkLimits()
    {
        float difference[2] = {abs(limitPosition.x - contactPointFromCenter.x),
                               abs(limitPosition.y - contactPointFromCenter.y)};
        float allowedDifference[2] = {abs(minimalStepDistance * (float)cos(walkingAngle) * (float)0.1),
                                      abs(minimalStepDistance * (float)sin(walkingAngle) * (float)0.1)};

        for (int i = 0; i < 2; i++)
            if (allowedDifference[i] < 0.001)
                allowedDifference[i] = 0.001;

        if (difference[0] <= allowedDifference[0] && difference[1] <= allowedDifference[1])
            return true;
        return false;
    }

    void move()
    {
        calculate_angles();

        write_to_servos();
    }
};
