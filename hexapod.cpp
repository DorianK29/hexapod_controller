#include <iostream>
#include <math.h>

#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>

#include "include/vector3.hpp"
#include "include/print.hpp"

webots::Robot *robot;
webots::Keyboard *input;

void readData();

void Wait(int time)
{
	int elapsed = 0;
	while (elapsed < time)
	{
		input->enable(time);
		robot->step(time);

		readData();

		elapsed += time;
	}
}

#include "include/hexapod.hpp"
#include "include/control.hpp"

int main(int argc, char **argv)
{
	robot = new webots::Robot();
	input = robot->getKeyboard();

	hexapod::init();

	hexapod::mainLoop();

	delete robot;
	return 0;
}