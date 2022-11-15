#include <iostream>
#include <hexapod.h>

/**
 * TODO:
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
string excludedTags[4] = {"wait", "movement", "limits", "calc"};
// DEBUG-END

float width, height;

float addedHeight = 10;
float motorSpeed = 10; // 0.37
int gaitStepMin = 5;
float walkingAngle = 0;
float walkingCadence = 30;

bool wait = true;

bool walkingDirection = false;		  // if true go forwards, if false go backwards
int mode;							  // what hexapod is doing rn
int sequence[6] = {3, 1, 6, 4, 2, 5}; // sequence of which the legs move when setting back to standing position
float limits[6][2];					  // 0 is x, 1 is y

leg &legSwitch(int num)
{
	switch (num)
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
}

void print(string output, string TAG = "empty")
{
	for (int i = 0; i < 4; i++)
		if (excludedTags[i] == TAG)
			return;

	cout << TAG + ": " << output << endl;
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
}

void initServos()
{
	for (int i = 1; i <= 6; i++)
	{
		leg *temp_leg = &legSwitch(i);
		for (int j = 1; j <= 3; j++)
		{
			temp_leg->servoSwitch(j).motor = robot->getMotor("servo " + to_string(i) + to_string(j));
			temp_leg->servoSwitch(j).motor->setVelocity(motorSpeed);
		}
	}
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
void servoWait(leg *Leg)
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

// wait for a given leg if given a legNum
// if not given anything wait for all legs
void legWait(int legNum = 0)
{
	if (legNum != 0) // for a given legNum
		servoWait(&legSwitch(legNum));
	else // for no given leg, ie. all legs
		for (int tempLegNum = 1; tempLegNum <= 6; tempLegNum++)
			servoWait(&legSwitch(tempLegNum));
}

// write calculated values to all servos of a leg
void legWrite(leg *Leg)
{
	// write calculated angles to the servo motors
	Leg->servo1.motor->setPosition(Leg->servo1.angle);
	Leg->servo2.motor->setPosition(Leg->servo2.angle);
	Leg->servo3.motor->setPosition(Leg->servo3.angle);

	Leg->servoRadToDeg(); // change calculated angle to degrees
	// print to serial
	print("Leg: " + to_string(getLegNum(Leg)) + " servo1: " + to_string(Leg->servo1.angle) + " servo2: " + to_string(Leg->servo2.angle) + " servo3: " + to_string(Leg->servo3.angle), "movement");
	Leg->servoDegToRad();

	// wait for the servo to reach the calculated angle
	if (wait)
		legWait(getLegNum(Leg));
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

void legValues();

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

void walk()
{
	print("started walking", "movement");
	bool limit = false;
	float gaitSteps = walkingCadence / gaitStepMin;
	while (!limit && mode == 3)
	{
		for (int legNum = 1; legNum <= 6; legNum++)
		{
			leg *local_leg = &legSwitch(legNum);
			bool legDirection = (legNum % 2 == 1) ? true : false; // legs number 1,3,5

			local_leg->l.x += -1 * pow(-1, legDirection) * pow(-1, walkingDirection) * cos(walkingAngle * DEG_TO_RAD) * walkingCadence / gaitSteps; // -1 because vector is backwards
			local_leg->l.y += -1 * pow(-1, legDirection) * pow(-1, walkingDirection) * sin(walkingAngle * DEG_TO_RAD) * walkingCadence / gaitSteps; // -1 because vector is backwards
			local_leg->l.z = (walkingDirection) ? height - addedHeight * legDirection : height - addedHeight * !legDirection;

			calculateM1(local_leg);

			print(to_string(legNum) + ": l.x=" + to_string(local_leg->l.x) + " l.y=" + to_string(local_leg->l.y) + " l.z" + to_string(local_leg->l.z), "calc");

			legCalc(local_leg);
			legWrite(local_leg);

			print(to_string(abs(local_leg->l.x)) + "-" + to_string(abs(limits[legNum][0])) + "=" + to_string(abs(local_leg->l.x) - abs(limits[legNum][0])) + " >= " + to_string(walkingCadence * cos(walkingAngle) / 2), "limits");
			print(to_string(abs(local_leg->l.y)) + "-" + to_string(abs(limits[legNum][1])) + "=" + to_string(abs(local_leg->l.x) - abs(limits[legNum][0])) + " >= " + to_string(walkingCadence * sin(walkingAngle) / 2), "limits");

			if (abs(local_leg->l.x) - abs(limits[legNum][0]) >= walkingCadence * cos(walkingAngle) / 2 && abs(local_leg->l.y) - abs(limits[legNum][1]) >= walkingCadence * sin(walkingAngle) / 2)
				limit = true;
		}
	}
	walkingDirection = !walkingDirection;
	print("ended walking", "movement");
}

// get vector l with values for current width, height and motor1 angle
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

// TODO:

// given a pos (either relative to current zero pos(m1 = 0, width distance, height distance), or absolute to body center)
// set values of vector l of given leg accordingly
void moveTo(leg *local_leg, vector3 pos, bool absolute = false)
{
	if (absolute)
	{
		local_leg->l.x = pos.x - local_leg->s1.x;
		local_leg->l.y = pos.y - local_leg->s1.y;
		local_leg->l.z = pos.z - local_leg->s1.z;
	}
	else
	{
		vector3 zero = get_l(local_leg, 0);
		local_leg->l.x = pos.x - zero.x;
		local_leg->l.y = pos.y - zero.y;
		local_leg->l.z = pos.z - zero.z;
	}

	calculateM1(local_leg);
	legCalc(local_leg);
	legWrite(local_leg);
}

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
			if (cycles == 0)
				for (int legNum = 1; legNum <= 6; legNum++)
				{
					leg *local_leg = &legSwitch(legNum);
					vector3 temp = get_l(local_leg, 0);
					limits[legNum][0] = temp.x;
					limits[legNum][1] = temp.y;
				}

			walk();

			cycles++;

			if (cycles >= 5)
				mode = 2;
			break;
		case 4:
			// TODO: idk fix bro
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