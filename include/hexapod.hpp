#pragma once

#include <webots/Motor.hpp>

extern const double DEG_TO_RAD;
extern const double RAD_TO_DEG;

namespace hexapod
{
    enum state
    {
        Idle = 0,
        Start,
        Stand,
        Walk,
        Rotate,
        Spin,
        Jump,
        Tilt,
        Balance
    };
    state currentState = state::Start;

#include "leg.hpp"
    leg leg1, leg2, leg3, leg4, leg5, leg6;
    leg *legIter[6] = {&leg1, &leg2, &leg3, &leg4, &leg5, &leg6};

    float width = 140, height = -50;
    float walkingAngle = 90;

    float swayModifier = 2;
    float breatheModifier = 0.5;

    void setRobotWalkingAngle();
    void switchLegGroundContact();
    void updateRobotRestPos();
#include "movement.hpp"

    void init();

    float stepDistance = 20;
    int minStepDistance = 5;
    float addedHeight = 10;

    int legMoveSequence[6] = {2, 0, 5, 3, 1, 4}; // sequence of which the legs calculateAnglesAndMove when setting back to standing position

    void angleFixForWebots()
    {
        legIter[2]->bodyServo.angleFix = 180 * DEG_TO_RAD;
        legIter[3]->bodyServo.angleFix = 180 * DEG_TO_RAD;
        legIter[4]->bodyServo.angleFix = 180 * DEG_TO_RAD;
        legIter[5]->bodyServo.angleFix = 360 * DEG_TO_RAD;
    }

    void connectServosToWebots()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->connectServoToWebots(legNum);
    }

    void updateLegsCompletedStepPosition()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->completedStepPosition = legIter[legNum]->defaultPosition + vector3{cos(walkingAngle) * stepDistance, sin(walkingAngle) * stepDistance, 0};
    }

    void setRobotWalkingAngle()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            if (legNum % 2 == 0)
                legIter[legNum]->movingAngle = walkingAngle * DEG_TO_RAD;
            else
                legIter[legNum]->movingAngle = (walkingAngle + 180) * DEG_TO_RAD;
        updateLegsCompletedStepPosition();
    }

    void setRobotStepDistance()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->stepDistance = stepDistance;
        updateLegsCompletedStepPosition();
    }

    void setRobotStepHeight()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->stepHeight = addedHeight;
    }

    void setRobotMinStepDistance()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->stepDistanceIncrement = minStepDistance;
    }

    void updateRobotRestPos()
    {
        for (int legNum = 0; legNum < 6; legNum++)
        {
            legIter[legNum]->walkingHeight = height;
            legIter[legNum]->walkingWidth = width;

            legIter[legNum]->defaultPosition = vector3{legIter[legNum]->walkingWidth * cos(legIter[legNum]->theta_z), legIter[legNum]->walkingWidth * sin(legIter[legNum]->theta_z), legIter[legNum]->walkingHeight};
        }
        updateLegsCompletedStepPosition();
    }

    void switchLegGroundContact()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->touchingGround = !legIter[legNum]->touchingGround;
    }

    void setLegsAttachmentPos()
    {
        for (int legNum = 0; legNum < 6; legNum++)
        {
            legIter[legNum]->theta_z = M_PI / 3 * legNum;
            legIter[legNum]->centerToBodyServo = {50 * (float)cos(M_PI / 3 * legNum), 50 * (float)sin(M_PI / 3 * legNum), -12.8};
        }
    }

    // wait for all legs
    void waitForAllLegs()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->waitUntilServosMove();
    }

    void init()
    {
        connectServosToWebots();

        setLegsAttachmentPos();

        for (int legNum = 0; legNum < 6; legNum++)
        {
            legIter[legNum]->stepDistanceIncrement = minStepDistance;
            legIter[legNum]->stepDistance = stepDistance;
            if (legNum % 2 == 0)
                legIter[legNum]->movingAngle = walkingAngle * DEG_TO_RAD;
            else
                legIter[legNum]->movingAngle = (walkingAngle + 180) * DEG_TO_RAD;
            legIter[legNum]->stepHeight = addedHeight;

            legIter[legNum]->touchingGround = (legNum % 2 == 1);
        }

        // setRobotMinStepDistance();
        // setRobotStepDistance();
        // setRobotWalkingAngle();
        // setRobotStepHeight();

        updateLegsCompletedStepPosition();
        updateRobotRestPos();

        angleFixForWebots();

        inertialUnit = Hexapod->getInertialUnit("inertial unit");
    }
};