#pragma once

#include <webots/Motor.hpp>

extern const double DEG_TO_RAD;
extern const double RAD_TO_DEG;

namespace hexapod
{
    enum state
    {
        idle = 0,
        start,
        stand,
        walk,
        rotate,
        spin,
        jump,
        tilt
    };
    state currentState = state::start;

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

    float stepLength = 20;
    int minStepDistance = 5;
    float addedHeight = 10;

    int legMoveSequence[6] = {2, 0, 5, 3, 1, 4}; // sequence of which the legs calculateAnglesAndMove when setting back to standing position

    void angleFix()
    {
        legIter[2]->bodyServo.angleFix = 180 * DEG_TO_RAD;
        legIter[3]->bodyServo.angleFix = 180 * DEG_TO_RAD;
        legIter[4]->bodyServo.angleFix = 180 * DEG_TO_RAD;
        legIter[5]->bodyServo.angleFix = 360 * DEG_TO_RAD;
    }

    void initAllServos()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->initServos(legNum);
    }

    void updateRobotLimitPos()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->completedStepPosition = legIter[legNum]->defaultPosition + vector3{cos(walkingAngle) * stepLength, sin(walkingAngle) * stepLength, 0};
    }

    void setRobotWalkingAngle()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            if (legNum % 2 == 0)
                legIter[legNum]->movingAngle = walkingAngle * DEG_TO_RAD;
            else
                legIter[legNum]->movingAngle = (walkingAngle + 180) * DEG_TO_RAD;
        updateRobotLimitPos();
    }

    void setRobotStepDistance()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->stepLength = stepLength;
        updateRobotLimitPos();
    }

    void setRobotStepHeight()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->stepHeight = addedHeight;
    }

    void setRobotMinStepDistance()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->stepLengthIncrement = minStepDistance;
    }

    void updateRobotRestPos()
    {
        for (int legNum = 0; legNum < 6; legNum++)
        {
            legIter[legNum]->walkingHeight = height;
            legIter[legNum]->walkingWidth = width;

            legIter[legNum]->defaultPosition = vector3{legIter[legNum]->walkingWidth * cos(legIter[legNum]->theta_z), legIter[legNum]->walkingWidth * sin(legIter[legNum]->theta_z), legIter[legNum]->walkingHeight};
        }
        updateRobotLimitPos();
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
    void robotWait()
    {
        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->waitUntilServosMove();
    }

    void init()
    {
        initAllServos();

        setLegsAttachmentPos();

        updateRobotRestPos();

        setRobotMinStepDistance();
        setRobotStepDistance();
        setRobotWalkingAngle();
        setRobotStepHeight();

        for (int legNum = 0; legNum < 6; legNum++)
            legIter[legNum]->touchingGround = (legNum % 2 == 1);

        angleFix();
    }

    void mainLoop()
    {
        int cycles = 0;

        while (true)
        {

            switch (currentState)
            {
            case state::idle:
                movement::breathe();
                for (int legNum = 0; legNum < 6; legNum++)
                    legIter[legNum]->calculateAnglesAndMove();
                for (int legNum = 0; legNum < 6; legNum++)
                    legIter[legNum]->waitUntilServosMove();
                Wait(20);
                break;
            case state::start:
                currentState = state::idle;

                for (int legNum = 0; legNum < 6; legNum++)
                {
                    legIter[legNum]->groundContactPoint = legIter[legNum]->defaultPosition + vector3{0, 0, -legIter[legNum]->defaultPosition.z};
                    legIter[legNum]->calculateAnglesAndMove();
                }
                Wait(500);

                for (int legNum = 0; legNum < 6; legNum++)
                {
                    legIter[legNum]->groundContactPoint = legIter[legNum]->defaultPosition;
                    legIter[legNum]->calculateAnglesAndMove();
                }
                Wait(500); // delay to make movement smoother
                break;
            case state::stand:
                currentState = state::idle;

                for (int sequenceIter = 0; sequenceIter < 6; sequenceIter++)
                {
                    int legNum = legMoveSequence[sequenceIter]; // go in a sequence different than 1->2->3...
                    legIter[legNum]->groundContactPoint = legIter[legNum]->groundContactPoint + vector3{0, 0, legIter[legNum]->stepHeight};
                    legIter[legNum]->calculateAnglesAndMove();
                    legIter[legNum]->waitUntilServosMove();
                    legIter[legNum]->groundContactPoint = legIter[legNum]->defaultPosition + vector3{0, 0, legIter[legNum]->stepHeight};
                    legIter[legNum]->calculateAnglesAndMove();
                    legIter[legNum]->waitUntilServosMove();
                    legIter[legNum]->groundContactPoint = legIter[legNum]->defaultPosition;
                    legIter[legNum]->calculateAnglesAndMove();
                    legIter[legNum]->waitUntilServosMove();
                }
                cycles = 0;
                break;
            case state::walk:
                movement::walk();

                // intermediate position
                for (int legNum = 0; legNum < 6; legNum++)
                    if (legIter[legNum]->touchingGround)
                    {
                        legIter[legNum]->groundContactPoint.z = legIter[legNum]->defaultPosition.z;
                        legIter[legNum]->calculateAnglesAndMove();
                        legIter[legNum]->waitUntilServosMove();
                    }

                cycles++;

                if (cycles >= 20)
                    currentState = state::stand;
                break;
            case state::rotate:
                movement::rotate(true, 360);

                currentState = state::stand;
                break;
            case state::spin:
                movement::spin();
                break;
            case state::jump:
                movement::jump();
                break;
            case state::tilt:
                movement::tilt();
                break;
            }
        }
    }
};