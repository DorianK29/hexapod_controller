#include <iostream>
#include <math.h>

#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>

#include "include/vector3.hpp"
#include "include/print.hpp"

webots::Robot *Hexapod;
webots::Keyboard *input;
webots::InertialUnit *inertialUnit;

void readData();

void Wait(int time)
{
	int elapsed = 0;
	while (elapsed < time)
	{
		input->enable(time);
		Hexapod->step(time);

		readData();

		elapsed += time;
	}
}

#include "include/hexapod.hpp"
#include "include/readData.hpp"

void mainLoop()
{
	using namespace hexapod;

	int cycles = 0;

	while (true)
	{

		switch (currentState)
		{
		case state::Idle:
			movement::breathe();
			for (int legNum = 0; legNum < 6; legNum++)
				legIter[legNum]->calculateAnglesAndMove();
			for (int legNum = 0; legNum < 6; legNum++)
				legIter[legNum]->waitUntilServosMove();
			Wait(20);
			break;
		case state::Start:
			// DEBUG:
			currentState = state::Balance;

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
		case state::Stand:
			currentState = state::Idle;

			cycles = 0;

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
			break;
		case state::Walk:
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
				currentState = state::Stand;
			break;
		case state::Rotate:
			movement::rotate(true, 360);

			currentState = state::Stand;
			break;
		case state::Spin:
			movement::spin();

			cycles++;

			if (cycles >= 2)
				currentState = state::Stand;
			break;
		case state::Jump:
			movement::jump();

			cycles++;

			if (cycles >= 2)
				currentState = state::Stand;
			break;
		case state::Tilt:
			movement::tilt();

			currentState = state::Stand;
			break;
		case state::Balance:
			movement::balance();

			break;
		}
	}
}

int main(int argc, char **argv)
{
	Hexapod = new webots::Robot();
	input = Hexapod->getKeyboard();

	hexapod::init();

	mainLoop();

	delete Hexapod;
	return 0;
}
