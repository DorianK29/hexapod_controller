#include <iostream>
#include <hexapod.h>

Robot *robot;
Keyboard *keyboard;

leg leg1, leg2, leg3, leg4, leg5, leg6;
leg legIter[6] = {leg1, leg2, leg3, leg4, leg5, leg6};

// DEBUG
// 1st bool is for empty
bool excludedTags[5] = {false, true, false, true, true};

int getTagIndex(string TAG)
{
	if (TAG == "wait")
		return 1;
	else if (TAG == "movement")
		return 2;
	else if (TAG == "limits")
		return 3;
	else if (TAG == "calc")
		return 4;
	return 0; // empty tag
}
// DEBUG-END

float width, height;

float motorSpeed = 10; // 0.37
float stepDistance = 35;
int minStepDistance = 5;
float addedHeight = 25;
float walkingAngle = 90;

bool wait = true;

int mode;							  // what hexapod is doing rn
int sequence[6] = {3, 1, 6, 4, 2, 5}; // sequence of which the legs move when setting back to standing position

leg &legSwitch(int legNum)
{
	switch (legNum)
	{
	case 1:
		return leg1;
		break;
	case 2:
		return leg2;
		break;
	case 3:
		return leg3;
		break;
	case 4:
		return leg4;
		break;
	case 5:
		return leg5;
		break;
	case 6:
		return leg6;
		break;
	}

	return leg1; // just so compiler throw no error :)
}

int getLegNum(leg *givenLeg)
{
	if (givenLeg == &leg1)
		return 1;
	else if (givenLeg == &leg2)
		return 2;
	else if (givenLeg == &leg3)
		return 3;
	else if (givenLeg == &leg4)
		return 4;
	else if (givenLeg == &leg5)
		return 5;
	else if (givenLeg == &leg6)
		return 6;
	return 0;
}

void initServos()
{
	for (int legNum = 0; legNum < 6; legNum++)
		for (int servoNum = 1; servoNum <= 3; servoNum++)
		{
			legIter[legNum].servoSwitch(servoNum).motor = robot->getMotor("servo " + to_string(legNum + 1) + to_string(servoNum));
			legIter[legNum].servoSwitch(servoNum).motor->setVelocity(motorSpeed);
		}
}

void updateRobotLimitPos()
{
	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].updateLimitPos();
}

void setRobotWalkingAngle()
{
	for (int legNum = 0; legNum < 6; legNum++)
		if (legNum % 2 == 0)
			legIter[legNum].walkingAngle = walkingAngle * DEG_TO_RAD;
		else
			legIter[legNum].walkingAngle = (walkingAngle + 180) * DEG_TO_RAD;
	updateRobotLimitPos();
}

void setRobotStepDistance()
{
	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].stepDistance = stepDistance;
	updateRobotLimitPos();
}

void setRobotStepHeight()
{
	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].stepHeight = addedHeight;
}

void setRobotMinStepDistance()
{
	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].minimalStepDistance = minStepDistance;
}

void updateRobotRestPos()
{
	for (int legNum = 0; legNum < 6; legNum++)
	{
		legIter[legNum].walkingHeight = height;
		legIter[legNum].walkingWidth = width;

		legIter[legNum].updateRestPos();
	}
	updateRobotLimitPos();
}

void switchLegGroundContact()
{
	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].touchingGround = !legIter[legNum].touchingGround;
}

/**
 * CONTROL
 */

void print(string output, string TAG = "empty")
{
	if (excludedTags[getTagIndex(TAG)])
		return;

	cout << TAG + ": " << output << endl;
	return;
}

void readData()
{
	int key = keyboard->getKey();

	if (key >= '0' && key <= '9')
	{
		width = 120 + 10 * (key - '0'); // 0 is 120, 9 is 210
		mode = 2;
		updateRobotRestPos();
		print("Width: " + to_string(width), "keyboard");
	}
	else if (key == 'W')
		mode = 3;
	else if (key == 'S')
		mode = 2;
	else if (key == 'A' || key == 'D')
	{
		walkingAngle += 4 * pow(-1, key);
		setRobotWalkingAngle();
		print("Walking Angle: " + to_string(walkingAngle), "keyboard");
	}
	else if (key == 'R')
		mode = 4;
	else if (key == 'O')
		mode = 5;
	else if (key == 'J')
		mode = 6;
	if (key != -1)
		print(to_string(key), "keyboard");
}

void angleFix()
{
	legIter[2].servo1.angleFix = 60 * DEG_TO_RAD;
	legIter[3].servo1.angleFix = 180 * DEG_TO_RAD;
	legIter[4].servo1.angleFix = 180 * DEG_TO_RAD;
	legIter[5].servo1.angleFix = 240 * DEG_TO_RAD;
}

/**
 * DELAY
 */

void Wait(int time)
{
	int elapsed = 0;
	while (elapsed < time)
	{
		keyboard->enable(time);
		robot->step(time);

		readData();

		elapsed += time;
	}
}

// wait for given leg
void legWait(leg Leg)
{
	int motorNum = 1;
	while (motorNum <= 3)
	{
		servo currentServo = Leg.servoSwitch(motorNum);
		PositionSensor *encoder = currentServo.motor->getPositionSensor();
		encoder->enable(1);
		Wait(1);
		float calcAngle = currentServo.angle;	   // calculated angle
		float servoAngle = encoder->getValue();	   // servo angle
		float difference = calcAngle - servoAngle; // difference between the calculated angle and the servo motor
		// if motor is not in position wait
		// a motor is in position if the difference is less than 1 degree
		if (abs(difference) >= 5 * DEG_TO_RAD)
			print("Servo Wait...", "wait"); // print wait if the difference is more than 1 degree
		else
			motorNum++; // when current motor is in position go to the next motor
		encoder->disable();
	}
}

// wait for all legs
void robotWait()
{
	for (int legNum = 1; legNum <= 6; legNum++)
		legWait(legSwitch(legNum));
}

/**
 * CALCULATION
 */

void setLegsAttachmentPos()
{
	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].setLegAttachmentPos(M_PI / 3 * legNum, {50 * (float)cos(M_PI / 3 * legNum), 50 * (float)sin(M_PI / 3 * legNum), -12.8});
}

/**
 * MOVEMENT
 */

void walk()
{
	print("started walking", "movement");
	bool limit = false;

	while (!limit && mode == 3)
	{
		for (int legNum = 0; legNum < 6; legNum++)
			legIter[legNum].contactPointFromCenter.z = (legIter[legNum].touchingGround) ? legIter[legNum].restingPosition.z : legIter[legNum].restingPosition.z + legIter[legNum].stepHeight;
		for (int legNum = 0; legNum < 6; legNum++)
		{
			legIter[legNum].contactPointFromCenter.x += cos(legIter[legNum].walkingAngle) * legIter[legNum].minimalStepDistance;
			legIter[legNum].contactPointFromCenter.y += sin(legIter[legNum].walkingAngle) * legIter[legNum].minimalStepDistance;

			legIter[legNum].move();

			legWait(legIter[legNum]);

			limit = legIter[legNum].checkLimits();
		}
	}

	walkingAngle += 180;
	switchLegGroundContact();
	setRobotWalkingAngle();

	print("ended walking", "movement");
}

// TODO: not finished
void spin()
{
	print("started spinning", "movement");

	float spinAngle = 360;
	float spinIncrement = 5;
	float allowedMovementAngle = 30;

	for (int j = 0; j < spinAngle / allowedMovementAngle && mode == 4; j++)
	{
		for (int legNum = 0; legNum < 6; legNum++)
		{
			legIter[legNum].contactPointFromCenter.z = (legIter[legNum].touchingGround) ? legIter[legNum].restingPosition.z : legIter[legNum].restingPosition.z + legIter[legNum].stepHeight;
		}
		for (int i = 0; i < allowedMovementAngle / spinIncrement; i++)
			for (int legNum = 0; legNum < 6; legNum++)
			{
				float angle = legIter[legNum].theta_z + legIter[legNum].servo1.angle + 90 * DEG_TO_RAD;
				angle += (legIter[legNum].touchingGround) ? M_PI : 0;
				cout << "Calculated angle: " << angle * RAD_TO_DEG << " thetaZ: " << legIter[legNum].theta_z * RAD_TO_DEG << "servo1 angle: " << legIter[legNum].servo1.angle * RAD_TO_DEG << endl;
				legIter[legNum].contactPointFromCenter.x += cos(angle) * spinIncrement;
				legIter[legNum].contactPointFromCenter.y += sin(angle) * spinIncrement;

				legIter[legNum].move();

				legWait(legIter[legNum]);
			}
		switchLegGroundContact();
	}

	print("ended spinning", "movement");
}

void rotatingDisc()
{
	float rotatingIncrement = 10;
	float currentPosition = 0;

	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].contactPointFromCenter.z = legIter[legNum].restingPosition.z + 20 * sin(legIter[legNum].theta_z) - 20;

	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].move();

	for (int legNum = 0; legNum < 6; legNum++)
		legWait(legIter[legNum]);

	while (mode == 5)
	{
		for (int legNum = 0; legNum < 6; legNum++)
		{
			legIter[legNum].contactPointFromCenter.z = legIter[legNum].restingPosition.z + 20 * sin(currentPosition + legIter[legNum].theta_z) + 10 * sin(currentPosition / 2) - 20;
			legIter[legNum].move();
			legWait(legIter[legNum]);
		}
		currentPosition += rotatingIncrement * DEG_TO_RAD;
	}
}

void jump()
{
	for (int legNum = 0; legNum < 6; legNum++)
	{
		width = 55;
		height = 0;
		updateRobotRestPos();
		legIter[legNum].contactPointFromCenter = legIter[legNum].restingPosition;
	}
	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].move();
	for (int legNum = 0; legNum < 6; legNum++)
		legWait(legIter[legNum]);
	Wait(200);
	for (int legNum = 0; legNum < 6; legNum++)
	{
		width = 52;
		height = 90;
		updateRobotRestPos();
		legIter[legNum].contactPointFromCenter = legIter[legNum].restingPosition;
	}
	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].move();
	for (int legNum = 0; legNum < 6; legNum++)
		legWait(legIter[legNum]);
	Wait(600);
}

/**
 * MAIN
 */

int main(int argc, char **argv)
{
	// create the Robot instance.
	robot = new Robot();
	keyboard = robot->getKeyboard();

	initServos();

	height = -50;
	width = 160;
	mode = 1;

	setLegsAttachmentPos();

	updateRobotRestPos();
	setRobotMinStepDistance();
	setRobotStepDistance();
	setRobotWalkingAngle();
	setRobotStepHeight();

	for (int legNum = 0; legNum < 6; legNum++)
		legIter[legNum].touchingGround = (legNum % 2 == 1);

	angleFix();

	int cycles = 0;

	// Main loop:
	// - perform simulation steps until Webots is stopping the controller
	while (robot->step(robot->getBasicTimeStep()) != -1)
	{
		switch (mode)
		{
		case 0:
			Wait(100);
			break;
		case 1: // start position
			mode = 0;
			wait = false;
			for (int legNum = 0; legNum < 6; legNum++)
			{
				legIter[legNum].contactPointFromCenter = legIter[legNum].restingPosition + vector3{0, 0, addedHeight};
				legIter[legNum].move();
			}
			Wait(500);
			for (int legNum = 0; legNum < 6; legNum++)
			{
				legIter[legNum].contactPointFromCenter = legIter[legNum].restingPosition;
				legIter[legNum].move();
			}
			wait = true;
			Wait(500); // delay to make movement smoother
			break;
		case 2: // stand
			mode = 0;
			for (int legNum = 0; legNum < 6; legNum++)
			{
				legIter[legNum].contactPointFromCenter = legIter[legNum].contactPointFromCenter + vector3{0, 0, legIter[legNum].stepHeight};
				legIter[legNum].move();
				legWait(legIter[legNum]);
				legIter[legNum].contactPointFromCenter = legIter[legNum].restingPosition + vector3{0, 0, legIter[legNum].stepHeight};
				legIter[legNum].move();
				legWait(legIter[legNum]);
				legIter[legNum].contactPointFromCenter = legIter[legNum].restingPosition;
				legIter[legNum].move();
				legWait(legIter[legNum]);
			}
			cycles = 0;
			break;
		case 3: // walk for a bit
			walk();

			// intermediate position
			for (int legNum = 0; legNum < 6; legNum++)
				if (legIter[legNum].touchingGround)
				{
					cout << "Intermediate: leg" << legNum << endl;
					cout << "contactPointFromCenter - x: " << legIter[legNum].contactPointFromCenter.x << ", y: " << legIter[legNum].contactPointFromCenter.y << ", z: " << legIter[legNum].contactPointFromCenter.z << endl;
					legIter[legNum].contactPointFromCenter.z = legIter[legNum].restingPosition.z;
					cout << "contactPointFromCenter - x: " << legIter[legNum].contactPointFromCenter.x << ", y: " << legIter[legNum].contactPointFromCenter.y << ", z: " << legIter[legNum].contactPointFromCenter.z << endl;
					legIter[legNum].move();
					legWait(legIter[legNum]);
				}

			cycles++;

			if (cycles >= 20)
				mode = 2;
			break;
		case 4: // spin
			spin();
			break;
		case 5: // rotate disc cool
			rotatingDisc();
			break;
		case 6: // jump
			jump();
			break;
		}
	}

	delete robot;
	return 0;
}
