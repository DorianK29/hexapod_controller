#pragma once

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>

#include "vector3.hpp"
#include "servo.hpp"

struct leg
{

    float lengthJoint1 = 44.75;
    float lengthJoint2 = 46.25;
    float lengthJoint3 = 65.7;

    servo bodyServo;
    servo kneeServo;
    servo ankleServo;
    servo *servoIter[3] = {&bodyServo, &kneeServo, &ankleServo};

    // Change with runtime
    // from the center of the body to the point of contact with the ground
    vector3 groundContactPoint;
    // from the center point of rotation of servo 1 to the point of contact with the ground
    vector3 bodyServoToGroundContactPoint;
    // from the center point of rotation of servo 2 to the point of contact with the ground
    vector3 kneeServoToGroundContactPoint;
    // from the center of the body to the center point of rotation of servo 1
    vector3 centerToBodyServo; // Doesn't change with runtime
    // from the center of the body to the center point of rotation of servo 2
    vector3 centerToKneeServo;
    // angle at which the leg joint is with reference to the body center
    float theta_z; // Doesn't change with runtime

    // same vector as groundContactPoint
    // the position of the leg when its standing/resting
    vector3 defaultPosition;
    // same vector as groundContactPoint
    // the farthest position the leg is allowed to be at
    vector3 completedStepPosition;

    // how much a leg moves in a single step
    float stepDistance;
    // the smallest increment of movement for the leg
    float stepDistanceIncrement;
    // how far up the leg moves while not in contact with the ground
    float stepHeight;

    // the angle at which the leg is moving
    float movingAngle;

    // is leg in contact with ground or not
    bool touchingGround;

    float walkingHeight, walkingWidth;

    // convert all servo angles from radians to degrees
    void radiansToDegrees()
    {
        bodyServo.radiansToDegrees();
        kneeServo.radiansToDegrees();
        ankleServo.radiansToDegrees();
    }
    // convert all servo angles from degrees to radians
    void degreesToRadians()
    {
        bodyServo.degreesToRadians();
        kneeServo.degreesToRadians();
        ankleServo.degreesToRadians();
    }

    void connectServoToWebots(int legNum)
    {
        for (int servoNum = 0; servoNum < 3; servoNum++)
        {
            servoIter[servoNum]->motor = Hexapod->getMotor("servo " + std::to_string(legNum + 1) + std::to_string(servoNum + 1));
            servoIter[servoNum]->motor->setVelocity(servoIter[servoNum]->motorSpeed);
        }
    }

    // void printValues()
    // {
    //     using namespace std;

    //     // print values for vector3 variables
    //     cout << "groundContactPoint - x: " << groundContactPoint.x << ", y: " << groundContactPoint.y << ", z: " << groundContactPoint.z << endl;
    //     cout << "bodyServoToGroundContactPoint - x: " << bodyServoToGroundContactPoint.x << ", y: " << bodyServoToGroundContactPoint.y << ", z: " << bodyServoToGroundContactPoint.z << endl;
    //     cout << "kneeServoToGroundContactPoint - x: " << kneeServoToGroundContactPoint.x << ", y: " << kneeServoToGroundContactPoint.y << ", z: " << kneeServoToGroundContactPoint.z << endl;
    //     cout << "centerToBodyServo - x: " << centerToBodyServo.x << ", y: " << centerToBodyServo.y << ", z: " << centerToBodyServo.z << endl;
    //     cout << "centerToKneeServo - x: " << centerToKneeServo.x << ", y: " << centerToKneeServo.y << ", z: " << centerToKneeServo.z << endl;
    //     cout << "defaultPosition - x: " << defaultPosition.x << ", y: " << defaultPosition.y << ", z: " << defaultPosition.z << endl;
    //     cout << "completedStepPosition - x: " << completedStepPosition.x << ", y: " << completedStepPosition.y << ", z: " << completedStepPosition.z << endl;

    //     // print values for float variables
    //     // cout << "theta_z - " << theta_z * RAD_TO_DEG << endl;
    //     // cout << "stepDistance - " << stepDistance << endl;
    //     // cout << "stepDistanceIncrement - " << stepDistanceIncrement << endl;
    //     cout << "movingAngle - " << movingAngle << endl;
    //     // cout << "walkingHeight - " << walkingHeight << endl;
    //     // cout << "walkingWidth - " << walkingWidth << endl;

    //     radiansToDegrees();
    //     // print values for motor variables
    //     cout << "servo1 - angle: " << bodyServo.angle << ", angleFix: " << bodyServo.angleFix * RAD_TO_DEG << endl;
    //     cout << "kneeServo - angle: " << kneeServo.angle << ", angleFix: " << kneeServo.angleFix * RAD_TO_DEG << endl;
    //     cout << "ankleServo - angle: " << ankleServo.angle << ", angleFix: " << ankleServo.angleFix * RAD_TO_DEG << endl;
    //     degreesToRadians();
    // }

    void calculateAngles()
    {
        bodyServoToGroundContactPoint = groundContactPoint - centerToBodyServo;

        bodyServo.angle = atan(bodyServoToGroundContactPoint.y / bodyServoToGroundContactPoint.x) - theta_z + bodyServo.angleFix;

        centerToKneeServo = centerToBodyServo + vector3{lengthJoint1 * cos(bodyServo.angle + theta_z), lengthJoint1 * sin(bodyServo.angle + theta_z), 0};

        kneeServoToGroundContactPoint = groundContactPoint - centerToKneeServo;

        float P = atan(abs(kneeServoToGroundContactPoint.z) / (sqrt(kneeServoToGroundContactPoint.x * kneeServoToGroundContactPoint.x + kneeServoToGroundContactPoint.y * kneeServoToGroundContactPoint.y)));
        float R = asin((abs(kneeServoToGroundContactPoint.z) - abs(bodyServoToGroundContactPoint.z)) / lengthJoint1);
        kneeServo.angle = -1 * (acos((pow(lengthJoint2, 2) + pow(kneeServoToGroundContactPoint.length(), 2) - pow(lengthJoint3, 2)) / (2 * lengthJoint2 * kneeServoToGroundContactPoint.length())) - (P + R));
        ankleServo.angle = -1 * (M_PI - acos((pow(lengthJoint2, 2) + pow(lengthJoint3, 2) - pow(kneeServoToGroundContactPoint.length(), 2)) / (2 * lengthJoint2 * lengthJoint3)));
    }

    void writeToServos()
    {
        bodyServo.motor->setPosition(bodyServo.angle);
        kneeServo.motor->setPosition(kneeServo.angle);
        ankleServo.motor->setPosition(ankleServo.angle);
    }

    // bool checkLimits()
    // {
    //     float difference[2] = {abs(limitPosition.x - groundContactPoint.x),
    //                            abs(limitPosition.y - groundContactPoint.y)};
    //     float allowedDifference[2] = {abs(stepDistanceIncrement * (float)cos(movingAngle) * (float)0.1),
    //                                   abs(stepDistanceIncrement * (float)sin(movingAngle) * (float)0.1)};

    //     for (int i = 0; i < 2; i++)
    //         if (allowedDifference[i] < 0.001)
    //             allowedDifference[i] = 0.001;

    //     if (difference[0] <= allowedDifference[0] && difference[1] <= allowedDifference[1])
    //         return true;
    //     return false;
    // }

    bool checkIfStepCompleted()
    {
        if ((defaultPosition - groundContactPoint).length() >= stepDistance)
            return true;
        return false;
    }

    void calculateAnglesAndMove()
    {
        calculateAngles();
        writeToServos();
    }

    // wait for given leg
    void waitUntilServosMove()
    {
        int servoNum = 0;
        while (servoNum < 3)
        {
            servo *currentServo = servoIter[servoNum];
            webots::PositionSensor *encoder = currentServo->motor->getPositionSensor();
            encoder->enable(1);
            Wait(1);
            float calcAngle = currentServo->angle;     // calculated angle
            float servoAngle = encoder->getValue();    // servo angle
            float difference = calcAngle - servoAngle; // difference between the calculated angle and the servo motor
            // if motor is not in position wait
            // a motor is in position if the difference is less than 1 degree
            if (abs(difference) >= 2 * DEG_TO_RAD)
                print((std::string) "Servo Wait...", tags::wait); // print wait if the difference is more than 1 degree
            else
                servoNum++; // when current motor is in position go to the next motor
            encoder->disable();
        }
    }

    void moveLeg(vector3 position)
    {
        if (position != nullVector)
            groundContactPoint = position;
        calculateAnglesAndMove();
        waitUntilServosMove();
    }
};