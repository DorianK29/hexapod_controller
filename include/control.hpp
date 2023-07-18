#pragma once

#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>

#include "hexapod.hpp"
#include "print.hpp"

extern webots::Keyboard *input;

void readData()
{
    int key = input->getKey();

    if (key >= '0' && key <= '9')
    {
        hexapod::width = 120 + 10 * (key - '0'); // 0 is 120, 9 is 210
        hexapod::currentState = hexapod::state::stand;
        hexapod::updateRobotRestPos();
        print("Width: " + std::to_string(hexapod::width), tags::keyboard);
    }
    else if (key == 'W')
        hexapod::currentState = hexapod::state::walk;
    else if (key == 'S')
        hexapod::currentState = hexapod::state::stand;
    else if (key == 'A' || key == 'D')
    {
        hexapod::walkingAngle += 0.1 * pow(-1, key + 1);
        hexapod::setRobotWalkingAngle();
        print("Walking Angle: " + std::to_string(hexapod::walkingAngle), tags::keyboard);
    }
    else if (key == 'R')
        hexapod::currentState = hexapod::state::rotate;
    else if (key == 'O')
        hexapod::currentState = hexapod::state::spin;
    else if (key == 'J')
        hexapod::currentState = hexapod::state::jump;
    else if (key == 'T')
        hexapod::currentState = hexapod::state::tilt;
    if (key != -1)
        print(std::to_string(key), tags::keyboard);
}