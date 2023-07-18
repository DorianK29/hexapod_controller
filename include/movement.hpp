#pragma once

#include <math.h>

#include "hexapod.hpp"

namespace movement
{
    void breathe()
    {
        float precision = 100;
        if (hexapod::breatheModifier > 0.4)
            hexapod::breatheModifier -= (float)(rand() % 10) / precision;
        else if (hexapod::breatheModifier < 0.6)
            hexapod::breatheModifier += (float)(rand() % 10) / precision;
        else
            hexapod::breatheModifier += pow(-1, (rand() % 2)) * (float)(rand() % 10) / precision;
        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->groundContactPoint.z += hexapod::breatheModifier * sin(robot->getTime() * 90 * DEG_TO_RAD);
    }

    void sway(float swayAngleDeg)
    {
        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->groundContactPoint.z += hexapod::swayModifier * sin(swayAngleDeg * DEG_TO_RAD + hexapod::legIter[legNum]->theta_z);
    }

    void walk()
    {
        bool endPositionReached = false;

        while (!endPositionReached && hexapod::currentState == hexapod::state::walk)
            for (int legNum = 0; legNum < 6; legNum++)
            {
                hexapod::legIter[legNum]->groundContactPoint.z = (hexapod::legIter[legNum]->touchingGround) ? hexapod::legIter[legNum]->defaultPosition.z : hexapod::legIter[legNum]->defaultPosition.z + hexapod::legIter[legNum]->stepHeight;
                breathe();

                if (hexapod::legIter[legNum]->touchingGround)
                {
                    hexapod::legIter[legNum]->groundContactPoint.x += cos(hexapod::legIter[legNum]->movingAngle) * hexapod::legIter[legNum]->stepLengthIncrement;
                    hexapod::legIter[legNum]->groundContactPoint.y += sin(hexapod::legIter[legNum]->movingAngle) * hexapod::legIter[legNum]->stepLengthIncrement;
                }
                else
                {
                    vector3 result = vector3{cos(hexapod::legIter[legNum]->movingAngle) * hexapod::legIter[legNum]->stepLength, sin(hexapod::legIter[legNum]->movingAngle) * hexapod::legIter[legNum]->stepLength, 0} - (hexapod::legIter[legNum]->groundContactPoint - hexapod::legIter[legNum]->defaultPosition);
                    hexapod::legIter[legNum]->groundContactPoint.x += result.x / (hexapod::legIter[legNum]->stepLength / hexapod::legIter[legNum]->stepLengthIncrement);
                    hexapod::legIter[legNum]->groundContactPoint.y += result.y / (hexapod::legIter[legNum]->stepLength / hexapod::legIter[legNum]->stepLengthIncrement);
                }

                hexapod::legIter[legNum]->calculateAnglesAndMove();

                hexapod::legIter[legNum]->waitUntilServosMove();

                endPositionReached = hexapod::legIter[legNum]->checkIfStepCompleted();
            }

        hexapod::walkingAngle += 180;
        hexapod::switchLegGroundContact();
        hexapod::setRobotWalkingAngle();
    }

    // TODO: not finished
    void rotate(bool direction, float rotationAmountDeg)
    {
        float spinIncrement = 5;
        float allowedMovementAngle = 30;

        for (int i = 0; i < rotationAmountDeg;)
        {
            bool endPositionReached = false;
            float currentAngle = 0;
            for (int legNum = 0; legNum < 6; legNum++)
                hexapod::legIter[legNum]->completedStepPosition = vector3{(float)cos(hexapod::legIter[legNum]->theta_z + pow(-1, direction) * allowedMovementAngle * DEG_TO_RAD / 2) * hexapod::width, (float)sin(hexapod::legIter[legNum]->theta_z + pow(-1, direction) * allowedMovementAngle * DEG_TO_RAD / 2) * hexapod::width, 0};
            std::cout << "Leg2 Limit - x: " << hexapod::legIter[2]->completedStepPosition.x << " y: " << hexapod::legIter[0]->completedStepPosition.y << std::endl;
            std::cout << "Leg2 Rest - x: " << hexapod::legIter[2]->defaultPosition.x << " y: " << hexapod::legIter[0]->defaultPosition.y << std::endl;
            while (hexapod::currentState == hexapod::state::rotate && !endPositionReached)
            {
                for (int legNum = 0; legNum < 6; legNum++)
                {
                    currentAngle += spinIncrement;

                    hexapod::legIter[legNum]->groundContactPoint = {hexapod::width * (float)cos(hexapod::legIter[legNum]->theta_z + pow(-1, direction) * currentAngle * DEG_TO_RAD),
                                                                    hexapod::width * (float)sin(hexapod::legIter[legNum]->theta_z + pow(-1, direction) * currentAngle * DEG_TO_RAD),
                                                                    (hexapod::legIter[legNum]->touchingGround) ? hexapod::legIter[legNum]->defaultPosition.z : hexapod::legIter[legNum]->defaultPosition.z + hexapod::legIter[legNum]->stepHeight};

                    hexapod::legIter[legNum]->calculateAnglesAndMove();

                    hexapod::legIter[legNum]->waitUntilServosMove();

                    endPositionReached = hexapod::legIter[legNum]->checkIfStepCompleted();
                }
            }
            hexapod::switchLegGroundContact();
            i += currentAngle;
            direction = !direction;
        }
    }

    void tilt()
    {
        float rotatingIncrement = 10;
        float currentPosition = 0;

        while (hexapod::currentState == hexapod::state::tilt)
        {
            for (int legNum = 0; legNum < 6; legNum++)
                hexapod::legIter[legNum]->groundContactPoint.z = hexapod::legIter[legNum]->defaultPosition.z + 20 * sin(currentPosition + hexapod::legIter[legNum]->theta_z);

            for (int legNum = 0; legNum < 6; legNum++)
                hexapod::legIter[legNum]->calculateAnglesAndMove();

            for (int legNum = 0; legNum < 6; legNum++)
                hexapod::legIter[legNum]->waitUntilServosMove();

            currentPosition += 90 * DEG_TO_RAD;
            Wait(2000);
        }
    }

    void spin()
    {
        float rotatingIncrement = 10;
        float currentPosition = 0;

        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->groundContactPoint.z = hexapod::legIter[legNum]->defaultPosition.z + 20 * sin(hexapod::legIter[legNum]->theta_z);

        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->calculateAnglesAndMove();

        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->waitUntilServosMove();

        while (hexapod::currentState == hexapod::state::spin)
        {
            for (int legNum = 0; legNum < 6; legNum++)
            {
                hexapod::legIter[legNum]->groundContactPoint.z = hexapod::legIter[legNum]->defaultPosition.z + 20 * sin(currentPosition + hexapod::legIter[legNum]->theta_z) + 10 * sin(currentPosition / 2);
                hexapod::legIter[legNum]->calculateAnglesAndMove();
                hexapod::legIter[legNum]->waitUntilServosMove();
            }
            currentPosition += rotatingIncrement * DEG_TO_RAD;
        }
    }

    void jump()
    {
        for (int legNum = 0; legNum < 6; legNum++)
        {
            hexapod::width = 55;
            hexapod::height = 0;
            hexapod::updateRobotRestPos();
            hexapod::legIter[legNum]->groundContactPoint = hexapod::legIter[legNum]->defaultPosition;
        }
        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->calculateAnglesAndMove();
        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->waitUntilServosMove();
        Wait(200);
        for (int legNum = 0; legNum < 6; legNum++)
        {
            hexapod::width = 52;
            hexapod::height = 90;
            hexapod::updateRobotRestPos();
            hexapod::legIter[legNum]->groundContactPoint = hexapod::legIter[legNum]->defaultPosition;
        }
        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->calculateAnglesAndMove();
        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->waitUntilServosMove();
        Wait(600);
    }
}