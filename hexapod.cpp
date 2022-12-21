#include <iostream>
#include <hexapod.h>

/**
 *
 * Features:
 * Print end pos of leg
 * Control via joystick
 * Change variables on the go
 * Stand at an angle
 *
 * Optimization:
 * Use smaller size variables
 * Clean up redundant code
 *
 */

Robot *robot;
Keyboard *keyboard;

leg leg1, leg2, leg3, leg4, leg5, leg6;

// DEBUG
// 1st bool is for empty
bool excludedTags[5] = {false, true, true, true, true};

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

float addedHeight = 10;
float motorSpeed = 10; // 0.37
int minStepDistance = 5;
float walkingAngle = 132;
float walkingCadence = 30;

bool wait = true;

bool robotWalkingDirection = false;	  // if true go forwards, if false go backwards
int mode;							  // what hexapod is doing rn
int sequence[6] = {3, 1, 6, 4, 2, 5}; // sequence of which the legs move when setting back to standing position

leg &legSwitch(int legNumber)
{
	switch (legNumber)
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
	for (int legNumber = 1; legNumber <= 6; legNumber++)
	{
		leg *temp_leg = &legSwitch(legNumber);
		for (int servoNumber = 1; servoNumber <= 3; servoNumber++)
		{
			temp_leg->servoSwitch(servoNumber).motor = robot->getMotor("servo " + to_string(legNumber) + to_string(servoNumber));
			temp_leg->servoSwitch(servoNumber).motor->setVelocity(motorSpeed);
		}
	}
}

// set stance of robot
void stance(char stance)
{
	switch (stance)
	{
	case 'n': // motor1 in the middle of its allowed values
		leg1.servo1.angle = 0;
		leg2.servo1.angle = 0;
		leg3.servo1.angle = 0;
		leg4.servo1.angle = 0;
		leg5.servo1.angle = 0;
		leg6.servo1.angle = 0;
		break;
	}
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
		print("Width: " + to_string(width), "keyboard");
	}
	else if (key == 'W')
		mode = 3;
	else if (key == 'S')
		mode = 2;
	else if (key == 'A' || key == 'D')
	{
		walkingAngle += 4 * pow(-1, key);
		print("Walking Angle: " + to_string(walkingAngle), "keyboard");
	}
	else if (key == 'J')
		mode = 4;
	if (key != -1)
		print(to_string(key), "keyboard");
}

/**
 * DELAY
 */

void Wait(int time)
{
	int elapsed = 0;
	int delay = 50;
	while (elapsed < time)
	{
		keyboard->enable(delay);
		robot->step(delay);

		readData();

		elapsed += delay;
	}
}

// wait for given leg
void legWait(leg *Leg)
{
	int motorNum = 1;
	while (motorNum <= 3)
	{
		servo currentServo = Leg->servoSwitch(motorNum);
		PositionSensor *encoder = currentServo.motor->getPositionSensor();
		encoder->enable(5);
		Wait(5);
		float calcAngle = currentServo.angle;	   // calculated angle
		float servoAngle = encoder->getValue();	   // servo angle
		float difference = calcAngle - servoAngle; // difference between the calculated angle and the servo motor
		// if motor is not in position wait
		// a motor is in position if the difference is less than 1 degree
		if (abs(difference) >= 1 * DEG_TO_RAD)
			print("Servo Wait...", "wait"); // print wait if the difference is more than 1 degree
		else
			motorNum++; // when current motor is in position go to the next motor
		encoder->disable();
	}
}

// wait for all legs
void robotWait()
{
	for (int legNumber = 1; legNumber <= 6; legNumber++)
		legWait(&legSwitch(legNumber));
}

/**
 * CALCULATION
 */

// get vector l with values for current width, height and motor1 angle
// could add + 2 * PI in all sine, never used with any motor1 angle different than 0
vector3 get_l(leg *local_leg, float motor1 = -99)
{
	vector3 l;
	if (motor1 == -99) // if custom motor1 value was not given
		motor1 = local_leg->servo1.angle;
	float x = sin(local_leg->angle + walkingAngle * DEG_TO_RAD);
	float y = sin(PI - (motor1 + local_leg->angle + walkingAngle * DEG_TO_RAD));
	x = round(x * 100000) / 100000;
	y = round(y * 100000) / 100000;
	if (abs(round(x)) == abs(round(y)))
	{
		l.x = -1 * cos(local_leg->angle + motor1) * (width - sqrt(pow(local_leg->s1.x, 2) + pow(local_leg->s1.y, 2)));
		l.y = -1 * sin(local_leg->angle + motor1) * (width - sqrt(pow(local_leg->s1.x, 2) + pow(local_leg->s1.y, 2)));
	}
	else
	{
		float temp = (width - sqrt(pow(local_leg->s1.x, 2) + pow(local_leg->s1.y, 2))) * x * y;
		l.x = -1 * cos(local_leg->angle + motor1) * temp;
		l.y = -1 * sin(local_leg->angle + motor1) * temp;
	}
	l.z = height;
	return l;
}

// given leg with preset vector3 l values and motor1 angle calculate the rest
void legCalc(leg *leg)
{
	leg->D.x = leg->l.x - leg->s1.x;
	leg->D.y = leg->l.y - leg->s1.y;
	leg->D.z = leg->l.z - leg->s1.z;

	leg->s2.x = leg->s1.x + joint1 * cos(leg->servo1.angle + leg->angle);
	leg->s2.y = leg->s1.y + joint1 * sin(leg->servo1.angle + leg->angle);
	leg->s2.z = leg->s1.z;

	leg->L.x = leg->D.x + leg->s2.x;
	leg->L.y = leg->D.y + leg->s2.y;
	leg->L.z = leg->D.z + leg->s2.z;

	float P = atan(abs(leg->L.z) / (sqrt(leg->L.x * leg->L.x + leg->L.y * leg->L.y)));
	float R = asin((abs(leg->L.z) - abs(leg->l.z)) / joint1);
	leg->servo2.angle = -1 * (acos((pow(joint2, 2) + pow(leg->L.length(), 2) - pow(joint3, 2)) / (2 * joint2 * leg->L.length())) - (P + R));
	leg->servo3.angle = -1 * (PI - acos((pow(joint2, 2) + pow(joint3, 2) - pow(leg->L.length(), 2)) / (2 * joint2 * joint3)));
	print("", "calc");
	print("Leg" + to_string(getLegNum(leg)), "calc");
	print("s1.x: " + to_string(leg->s1.x) + " s1.y: " + to_string(leg->s1.y) + " s1.z: " + to_string(leg->s1.z), "calc");
	print("s2.x: " + to_string(leg->s2.x) + " s2.y: " + to_string(leg->s2.y) + " s2.z: " + to_string(leg->s2.z), "calc");
	print("l.x: " + to_string(leg->l.x) + " l.y: " + to_string(leg->l.y) + " l.z: " + to_string(leg->l.z), "calc");
	print("L.x: " + to_string(leg->L.x) + " L.y: " + to_string(leg->L.y) + " L.z: " + to_string(leg->L.z), "calc");
	print("D.x: " + to_string(leg->D.x) + " D.y: " + to_string(leg->D.y) + " D.z: " + to_string(leg->D.z), "calc");
	print("m1: " + to_string(leg->servo1.angle) + " m2: " + to_string(leg->servo2.angle) + " m3: " + to_string(leg->servo3.angle), "calc");

	print("", "calc");
}

void calculateM1(leg *Leg)
{
	int legNum = getLegNum(Leg);
	// M1 angle fix for webots
	if (legNum == 1 || legNum == 4)
		Leg->servo1.angle = atan(Leg->l.y / Leg->l.x); // find the correct formula -- obsidian note
	else if (legNum == 2 || legNum == 5)
		Leg->servo1.angle = atan(Leg->l.y / Leg->l.x) - PI / 3; // find the correct formula -- obsidian note
	else if (legNum == 3 || legNum == 6)
		Leg->servo1.angle = atan(Leg->l.y / Leg->l.x) + PI / 3; // find the correct formula -- obsidian note
}

void legValues()
{
	for (int legNum = 1; legNum <= 6; legNum++)
	{
		leg *local_leg = &legSwitch(legNum);
		local_leg->angle = PI / 3 * (legNum - 1);
		local_leg->s1.x = 50 * cos(local_leg->angle);
		local_leg->s1.y = 50 * sin(local_leg->angle);
		local_leg->s1.z = -12.8;
	}
}

bool checkLimits(leg *Leg)
{
	int legNum = getLegNum(Leg) - 1;

	vector3 positionZero = get_l(Leg, 0);

	print("L.x: " + to_string(Leg->l.x) + " L.x (0): " + to_string(positionZero.x) + " diff: " + to_string(abs(Leg->l.x - positionZero.x)) + " allowed diff: " + to_string(abs(walkingCadence * cos(walkingAngle * DEG_TO_RAD) / 2)), "limits");
	print("L.y: " + to_string(Leg->l.y) + " L.y (0): " + to_string(positionZero.y) + " diff: " + to_string(abs(Leg->l.y - positionZero.y)) + " allowed diff: " + to_string(abs(walkingCadence * sin(walkingAngle * DEG_TO_RAD) / 2)), "limits");

	if (abs(Leg->l.x - positionZero.x) >= abs(walkingCadence * cos(walkingAngle * DEG_TO_RAD) / 2) || abs(Leg->l.y - positionZero.y) >= abs(walkingCadence * sin(walkingAngle * DEG_TO_RAD) / 2))
		return true;
	return false;
}

/**
 * MOVEMENT
 */

// write calculated values to all servos of a leg
void legWrite(leg *Leg)
{
	// write calculated angles to the servo motors
	Leg->servo1.motor->setPosition(Leg->servo1.angle);
	Leg->servo2.motor->setPosition(Leg->servo2.angle);
	Leg->servo3.motor->setPosition(Leg->servo3.angle);

	// wait for the servo to reach the calculated angle
	if (wait)
		legWait(Leg);
}

/**
 * mode='a' - absolute (to body)
 *		'p' - add to current pos
		'n' - subtract from current pos
 */
// set values of vector l of given leg accordingly
void moveTo(leg *local_leg, vector3 pos, char mode = 'a')
{
	switch (mode)
	{
	case 'a':
		local_leg->l.x = pos.x - local_leg->s1.x;
		local_leg->l.y = pos.y - local_leg->s1.y;
		local_leg->l.z = pos.z - local_leg->s1.z;
		break;
	case 'p': // add to current pos
			  // vector l is going in the negative direction
		local_leg->l.x -= pos.x;
		local_leg->l.y -= pos.y;
		local_leg->l.z -= pos.z;
		break;
	case 'n': // subtract from current pos
			  // vector l is going in the negative direction
		local_leg->l.x += pos.x;
		local_leg->l.y += pos.y;
		local_leg->l.z += pos.z;
		break;
	}

	int legNum = getLegNum(local_leg);
	print(to_string(legNum) + ": l.x=" + to_string(local_leg->l.x) + " l.y=" + to_string(local_leg->l.y) + " l.z" + to_string(local_leg->l.z), "calc");
	calculateM1(local_leg);
	legCalc(local_leg);
	legWrite(local_leg);
}

void walk()
{
	print("started walking", "movement");
	bool limit = false;
	float gaitSteps = walkingCadence / minStepDistance;
	while (!limit && mode == 3)
	{
		for (int legNum = 1; legNum <= 6; legNum++)
		{
			leg *local_leg = &legSwitch(legNum);
			bool legMovingDirection = (legNum % 2 == 1) ? true : false; // legs number 1,3,5

			local_leg->l.x += -1 * pow(-1, legMovingDirection) * pow(-1, robotWalkingDirection) * cos(walkingAngle * DEG_TO_RAD) * walkingCadence / gaitSteps; // -1 because vector is backwards
			local_leg->l.y += -1 * pow(-1, legMovingDirection) * pow(-1, robotWalkingDirection) * sin(walkingAngle * DEG_TO_RAD) * walkingCadence / gaitSteps; // -1 because vector is backwards
			local_leg->l.z = (robotWalkingDirection) ? height - addedHeight * legMovingDirection : height - addedHeight * !legMovingDirection;

			calculateM1(local_leg);

			print(to_string(legNum) + ": l.x=" + to_string(local_leg->l.x) + " l.y=" + to_string(local_leg->l.y) + " l.z" + to_string(local_leg->l.z), "calc");

			legCalc(local_leg);
			legWrite(local_leg);

			limit = checkLimits(local_leg);
		}
	}
	robotWalkingDirection = !robotWalkingDirection;
	print("ended walking", "movement");
}

void walk2()
{
	print("started walking", "movement");
	bool limit = false;
	float gaitSteps = walkingCadence / minStepDistance;
	while (!limit && mode == 3)
	{
		for (int legNum = 1; legNum <= 6; legNum++)
		{
			leg *local_leg = &legSwitch(legNum);
			bool legMovingDirection = (legNum % 2 == 1) ? true : false; // legs number 1,3,5
			vector3 pos = {cos(walkingAngle * DEG_TO_RAD) * walkingCadence / gaitSteps,
						   sin(walkingAngle * DEG_TO_RAD) * walkingCadence / gaitSteps,
						   (robotWalkingDirection) ? -addedHeight / gaitSteps * legMovingDirection : -addedHeight / gaitSteps * !legMovingDirection};

			// This bool is equal to legMovingDirection, but if robotWalkingDirection is true than flip the bool
			moveTo(local_leg, pos, ((legMovingDirection * !robotWalkingDirection) || (!legMovingDirection * robotWalkingDirection)) ? 'p' : 'n');

			limit = checkLimits(local_leg);
		}
	}
	robotWalkingDirection = !robotWalkingDirection;
	print("ended walking", "movement");
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
	legValues();

	height = 50;
	width = 160;
	mode = 1;

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
			stance('n'); // set motor1 angles to their center value
			wait = false;
			for (int i = 1; i <= 6; i++)
			{
				leg *local_leg = &legSwitch(i);
				local_leg->l = get_l(local_leg);
				local_leg->l.z = 13; // set height of walking to 20 above of desired point so legs dont drag on the ground
				legCalc(local_leg);	 // calculate motor2 and motor3 angles
				legWrite(local_leg); // write calculated values to leg servos
			}
			Wait(500);
			for (int i = 0; i < 6; i++)
			{
				leg *local_leg = &legSwitch(sequence[i]);
				local_leg->l.z = height; // set height of walking to desired value
				legCalc(local_leg);		 // calculate motor2 and motor3 angles
				legWrite(local_leg);
			}
			wait = true;
			Wait(500); // delay to make movement smoother
			break;
		case 2: // stand
			mode = 0;
			stance('n'); // set motor1 angles to their center value
			for (int i = 0; i < 6; i++)
			{
				leg *local_leg = &legSwitch(sequence[i]);
				local_leg->l = get_l(local_leg);
				legCalc(local_leg);		 // calculate motor2 and motor3 angles
				legWrite(local_leg);	 // write calculated values to leg servos
				local_leg->l.z = height; // set height of walking to desired value
				legCalc(local_leg);		 // calculate motor2 and motor3 angles
				legWrite(local_leg);	 // write calculated values to leg servos
				Wait(200);				 // delay to make movement smoother
			}
			cycles = 0;
			break;
		case 3: // walk for a bit
			walk2();

			cycles++;

			if (cycles >= 5)
				mode = 2;
			break;
		case 4:
			vector3 pos = {30, 30, height};
			cout << &leg1 << endl;
			print(to_string(pos.x) + " " + to_string(pos.y) + " " + to_string(pos.z) + " ");
			moveTo(&leg1, pos);
			mode = 0;
			break;
		}
	}

	delete robot;
	return 0;
}
