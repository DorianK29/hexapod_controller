#pragma once

#include <math.h>

#include "hexapod.hpp"

namespace movement
{
    void adjustWalkingWidthHeight()
    {
        for (int legNum = 0; legNum < 6; legNum++)
        {
            float difference[2] = {abs(hexapod::legIter[legNum]->groundContactPoint.lengthXY() - hexapod::legIter[legNum]->kneeServoToGroundContactPoint.lengthXY()), abs(abs(hexapod::legIter[legNum]->groundContactPoint.z) - abs(hexapod::legIter[legNum]->kneeServoToGroundContactPoint.z))};
            if (difference[0] <= 15)
            {
                std::cout << "Width 2 Small: " << hexapod::width;
                hexapod::width += 5;
                hexapod::updateRobotRestPos();
                std::cout << " -> " << hexapod::width << std::endl;

                break;
            }
            else if (difference[0] >= 150)
            {
                std::cout << "Width 2 Big: " << hexapod::width;
                hexapod::width -= 5;
                hexapod::updateRobotRestPos();
                std::cout << " -> " << hexapod::width << std::endl;
                break;
            }

            // TODO: check if  difference[1] is negative and if it sohuld be -15 and -70
            if (difference[1] <= 15)
            {
                std::cout << "Height 2 Big: " << hexapod::height;
                hexapod::height -= 5;
                hexapod::updateRobotRestPos();
                std::cout << " -> " << hexapod::height << std::endl;
                break;
            }
            else if (difference[1] >= 70)
            {
                std::cout << "Height 2 Small: " << hexapod::height;
                hexapod::height += 5;
                hexapod::updateRobotRestPos();
                std::cout << " -> " << hexapod::height << std::endl;
                break;
            }
        }
    }

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
            hexapod::legIter[legNum]->groundContactPoint.z += hexapod::breatheModifier * sin(Hexapod->getTime() * 90 * DEG_TO_RAD);
    }

    void sway(float swayAngleDeg)
    {
        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->groundContactPoint.z += hexapod::swayModifier * sin(swayAngleDeg * DEG_TO_RAD + hexapod::legIter[legNum]->theta_z);
    }

    void walk2()
    {
        bool endPositionReached = false;

        for (int state = 0; state < 3; state++)
            for (int legNum = 0; legNum < 6; legNum++)
            {
                // change z to sin function with respect to time increment
                // hexapod::legIter[legNum]->groundContactPoint.z = () ? hexapod::legIter[legNum]->defaultPosition.z : hexapod::legIter[legNum]->defaultPosition.z + hexapod::legIter[legNum]->stepHeight;
                breathe();

                hexapod::legIter[legNum]->groundContactPoint.x += cos(hexapod::legIter[legNum]->movingAngle) * hexapod::legIter[legNum]->stepDistanceIncrement;
                hexapod::legIter[legNum]->groundContactPoint.y += sin(hexapod::legIter[legNum]->movingAngle) * hexapod::legIter[legNum]->stepDistanceIncrement;

                hexapod::legIter[legNum]->calculateAnglesAndMove();

                hexapod::legIter[legNum]->waitUntilServosMove();

                endPositionReached = hexapod::legIter[legNum]->checkIfStepCompleted();
            }

        hexapod::walkingAngle += 180;
        hexapod::setRobotWalkingAngle();
    }

    void walk()
    {
        bool endPositionReached = false;

        std::vector<std::string> output[6][4];

        while (!endPositionReached && hexapod::currentState == hexapod::state::Walk)
        {
            for (int legNum = 0; legNum < 6; legNum++)
            {
                // change z to sin function with respect to time increment
                hexapod::legIter[legNum]->groundContactPoint.z = (hexapod::legIter[legNum]->touchingGround) ? hexapod::legIter[legNum]->defaultPosition.z : hexapod::legIter[legNum]->defaultPosition.z + hexapod::legIter[legNum]->stepHeight;
                breathe();

                hexapod::legIter[legNum]->groundContactPoint.x += cos(hexapod::legIter[legNum]->movingAngle) * hexapod::legIter[legNum]->stepDistanceIncrement;
                hexapod::legIter[legNum]->groundContactPoint.y += sin(hexapod::legIter[legNum]->movingAngle) * hexapod::legIter[legNum]->stepDistanceIncrement;

                hexapod::legIter[legNum]->calculateAnglesAndMove();

                hexapod::legIter[legNum]->waitUntilServosMove();

                endPositionReached = hexapod::legIter[legNum]->checkIfStepCompleted();
            }

            for (int legNum = 0; legNum < 6; legNum++)
            {
                output[legNum][0].push_back("Leg " + std::to_string(legNum + 1));
                for (int servoNum = 0; servoNum < 3; servoNum++)
                {
                    hexapod::legIter[legNum]->servoIter[servoNum]->motor->enableTorqueFeedback(1);
                    float currentTorque = hexapod::legIter[legNum]->servoIter[servoNum]->motor->getTorqueFeedback();
                    output[legNum][servoNum + 1].push_back(std::to_string(currentTorque));
                }
            }
        }

        printf("%6s %15s %15s %15s\n", "Leg", "Servo 1", "Servo 2", "Servo3");
        for (int legNum = 0; legNum < 6; legNum++)
            for (int i = 0; i < output[0]->size(); i++)
                printf("%6s %15s %15s %15s\n", output[legNum][0][i].c_str(), output[legNum][1][i].c_str(), output[legNum][2][i].c_str(), output[legNum][3][i].c_str());

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
                hexapod::legIter[legNum]->completedStepPosition = vector3{hexapod::width * (float)cos(hexapod::legIter[legNum]->theta_z + pow(-1, direction) * allowedMovementAngle * DEG_TO_RAD / 2), hexapod::width * (float)sin(hexapod::legIter[legNum]->theta_z + pow(-1, direction) * allowedMovementAngle * DEG_TO_RAD / 2), 0};
            while (hexapod::currentState == hexapod::state::Rotate && !endPositionReached)
            {
                for (int legNum = 0; legNum < 6; legNum++)
                {
                    currentAngle += spinIncrement;

                    hexapod::legIter[legNum]->groundContactPoint = {hexapod::width * (float)cos(hexapod::legIter[legNum]->theta_z + pow(-1, direction) * currentAngle * DEG_TO_RAD + hexapod::legIter[legNum]->bodyServo.angleFix),
                                                                    hexapod::width * (float)sin(hexapod::legIter[legNum]->theta_z + pow(-1, direction) * currentAngle * DEG_TO_RAD + hexapod::legIter[legNum]->bodyServo.angleFix),
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

        for (int i = 0; i < 4; i++)
        {
            for (int legNum = 0; legNum < 6; legNum++)
                hexapod::legIter[legNum]->groundContactPoint.z = hexapod::legIter[legNum]->defaultPosition.z + 20 * sin(currentPosition + hexapod::legIter[legNum]->theta_z);

            for (int legNum = 0; legNum < 6; legNum++)
                hexapod::legIter[legNum]->calculateAnglesAndMove();

            for (int legNum = 0; legNum < 6; legNum++)
                hexapod::legIter[legNum]->waitUntilServosMove();

            currentPosition += 90 * DEG_TO_RAD;
            Wait(1000);
        }
    }

    void balance()
    {
        double rotationChange[3] = {0};

        while (true)
        {
            inertialUnit->enable(10);
            rotationChange[0] += inertialUnit->getRollPitchYaw()[0] - rotationChange[0];
            rotationChange[1] += inertialUnit->getRollPitchYaw()[1] - rotationChange[1];
            rotationChange[2] += inertialUnit->getRollPitchYaw()[2] - rotationChange[2];
            inertialUnit->disable();

            // printf("RotationChange X rotation %4f Y rotation %4f Z rotation %4f\n", rotationChange[0], rotationChange[1], rotationChange[2]);

            inertialUnit->enable(10);
            for (int legNum = 0; legNum < 6; legNum++)
                hexapod::legIter[legNum]->groundContactPoint.z += hexapod::legIter[legNum]->defaultPosition.y * tan(rotationChange[0]) - hexapod::legIter[legNum]->defaultPosition.x * tan(rotationChange[1]);
            for (int legNum = 0; legNum < 6; legNum++)
                hexapod::legIter[legNum]->calculateAnglesAndMove();
            for (int legNum = 0; legNum < 6; legNum++)
                hexapod::legIter[legNum]->waitUntilServosMove();

            rotationChange[0] += inertialUnit->getRollPitchYaw()[0];
            rotationChange[1] += inertialUnit->getRollPitchYaw()[1];
            rotationChange[2] += inertialUnit->getRollPitchYaw()[2];
            inertialUnit->disable();

            adjustWalkingWidthHeight();
        }
    }

    void spin()
    {
        float rotatingIncrement = 5;
        float currentPosition = 0;

        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->groundContactPoint.z = hexapod::legIter[legNum]->defaultPosition.z + 20 * sin(hexapod::legIter[legNum]->theta_z);

        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->calculateAnglesAndMove();

        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->waitUntilServosMove();

        for (int i = 0; i < 360 / rotatingIncrement; i++)
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
        float temp[2] = {hexapod::width, hexapod::height};
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

        hexapod::width = temp[0];
        hexapod::height = temp[1];
        hexapod::updateRobotRestPos();
        for (int legNum = 0; legNum < 6; legNum++)
            hexapod::legIter[legNum]->groundContactPoint = hexapod::legIter[legNum]->defaultPosition;
    }
}