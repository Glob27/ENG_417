#include "kinematics.h"
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cstdio>

Taskspace homePos = { 125, 0, 20, (-90 * (PI / 180)), 0, 0 };
Taskspace lastTask = homePos;

double RadtoDeg(double rad)
{
	return rad * (180.0 / PI);
}

int Microbot::InverseKinematics(Taskspace t, Jointspace& j)
{
	printf("inside InverseKinematics\n");
	fflush(stdout);

	// Extract taskspace values
	double px = t.x;
	double py = t.y;
	double pz = t.z;
	double p = t.p;
	double r = t.r;

	// Joint definitions from task space
	double theta1 = std::atan2(py, px);
	double theta234 = p + (PI / 2.0);
	double theta5 = r;

	// Trig
	double c1 = std::cos(theta1);
	double s1 = std::sin(theta1);
	double c234 = std::cos(theta234);
	double s234 = std::sin(theta234);

	// Wrist center
	double Wx = px - d * c1 * s234;
	double Wy = py - d * s1 * s234;
	double Wz = pz + d * c234;

	// Solve theta3
	double c3_raw = ((Wx * Wx) + (Wy * Wy) + ((Wz - h) * (Wz - h))) / (2.0 * (a * a)) - 1.0;

	if (c3_raw > 1.0 || c3_raw < -1.0) {
		std::cout << "IK ERROR: Position out of reach\n";
		return 0;
	}

	double c3 = std::max(-1.0, std::min(1.0, c3_raw));

	if (std::fabs(1.0 + c3) < 1e-6) {
		std::cout << "IK WARNING: singular configuration\n";
		return 0;
	}

	// Elbow-down solution
	double s3 = -std::sqrt(1.0 - (c3 * c3));
	double theta3 = std::atan2(s3, c3);

	// Solve theta2
	double planar = std::sqrt((Wx * Wx) + (Wy * Wy));

	double c2, s2;

	if (Wx >= 0.0) {
		c2 = ((Wz - h) * s3 + planar * (1.0 + c3)) / (2.0 * a * (1.0 + c3));
		s2 = ((Wz - h) * (1.0 + c3) - s3 * planar) / (2.0 * a * (1.0 + c3));
	}
	else {
		c2 = ((Wz - h) * s3 - planar * (1.0 + c3)) / (2.0 * a * (1.0 + c3));
		s2 = ((Wz - h) * (1.0 + c3) + s3 * planar) / (2.0 * a * (1.0 + c3));
	}

	double theta2 = std::atan2(s2, c2);

	// Solve theta4 so total pitch matches theta234
	double theta4 = theta234 - theta2 - theta3;

	// Store joint values
	j.t[0] = theta1;
	j.t[1] = theta2;
	j.t[2] = theta3;
	j.t[3] = theta4; // wrist pitch contribution
	j.t[4] = theta5; // wrist roll

	printf("angles the microbot is using %.2f %.2f %.2f %.2f %.2f\n",
		RadtoDeg(theta1),
		RadtoDeg(theta2),
		RadtoDeg(theta3),
		RadtoDeg(theta4),
		RadtoDeg(theta5));
	fflush(stdout);

	return 1;
}

int AngleToSteps(int motor, double angleRad)
{
	static const double stepsPerRad[] = {
		BASE_STEPS / (2.0 * PI), // Motor 1 (Base)
		SHOULDER_STEPS / (2.0 * PI), // Motor 2 (Shoulder)
		ELBOW_STEPS / (2.0 * PI),  // Motor 3 (Elbow)
		RIGHT_STEPS / (2.0 * PI),  // Motor 4 (Wrist differential side A)
		LEFT_STEPS / (2.0 * PI),  // Motor 5 (Wrist differential side B)
		GRIPPER_STEPS / 28.0   // Motor 6 (Gripper placeholder)
	};

	// Software-only direction correction
	// Flip these signs if any axis still moves opposite
	static const int motorSign[] = {
		0,   // unused
		1,   // Motor 1
		-1,  // Motor 2
		-1,  // Motor 3
		-1,   // Motor 4
		-1,   // Motor 5
		1    // Motor 6
	};

	return static_cast<int>(std::round(motorSign[motor] * angleRad * stepsPerRad[motor - 1]));
}

//this is part of an output rework that lets us see the number of steps the robot takes
//itll be deleted at the end
double StepsToAngle(int motor, int steps) 
{
	static const double radPerStep[] = {
		(2.0 * PI) / BASE_STEPS,
		(2.0 * PI) / SHOULDER_STEPS,
		(2.0 * PI) / ELBOW_STEPS,
		(2.0 * PI) / RIGHT_STEPS,
		(2.0 * PI) / LEFT_STEPS,
		1.0
	};

	static const int motorSign[] = {
		0,
		1,
		-1,
		-1,
		-1,
		-1,
		1
	};

	return (steps * radPerStep[motor - 1]) / motorSign[motor];
}

int Microbot::ForwardKinematics(Jointspace j, Taskspace& t)
{
	// Joint angles
	double theta1 = j.t[0];
	double theta2 = j.t[1];
	double theta3 = j.t[2];
	double theta4 = j.t[3];
	double theta5 = j.t[4];

	// Combined angles
	double theta23 = theta2 + theta3;
	double theta234 = theta2 + theta3 + theta4;

	// Trig
	double c1 = std::cos(theta1);
	double s1 = std::sin(theta1);

	double c2 = std::cos(theta2);
	double s2 = std::sin(theta2);

	double c23 = std::cos(theta23);
	double s23 = std::sin(theta23);

	double c234 = std::cos(theta234);
	double s234 = std::sin(theta234);

	// Position
	t.x = c1 * (a * c2 + a * c23 + d * s234);
	t.y = s1 * (a * c2 + a * c23 + d * s234);
	t.z = h + a * s2 + a * s23 - d * c234;

	// Orientation
	t.p = theta234 - (PI / 2.0);
	t.r = theta5;

	return 1;
}

int mmToStepsGrip(double grip)
{
	return static_cast<int>(std::round(grip * 13.4));
}

//anouther debugging tool thatll get got once were done
double stepsToMmGrip(int steps)
{
	return steps / 13.4;
}

int Microbot::MoveTo(Taskspace& t, int speed)
{
	Jointspace currentJoint = {};
	Jointspace targetJoint = {};
	Registerspace delta = {};
	Taskspace achievedTask = {};

	// Solve current remembered pose
	if (!InverseKinematics(lastTask, currentJoint))
	{
		std::cout << "MoveTo ERROR: could not solve IK for starting position.\n";
		return 0;
	}

	// Solve target pose
	if (!InverseKinematics(t, targetJoint))
	{
		std::cout << "MoveTo ERROR: could not solve IK for target position.\n";
		return 0;
	}

	// Joint deltas in radians
	double dt1 = targetJoint.t[0] - currentJoint.t[0];
	double dt2 = targetJoint.t[1] - currentJoint.t[1];
	double dt3 = targetJoint.t[2] - currentJoint.t[2];
	double dt4 = targetJoint.t[3] - currentJoint.t[3];
	double dt5 = targetJoint.t[4] - currentJoint.t[4];
	double dg = t.g - lastTask.g;

	// Use coupled motor math like the better-working file
	delta.r[1] = AngleToSteps(1, dt1);

	// Shoulder motor
	delta.r[2] = AngleToSteps(2, dt2);

	// Elbow motor carries shoulder + elbow coupling
	delta.r[3] = AngleToSteps(3, dt2 + dt3);

	// Wrist differential motors:
	// motor 4 = total pitch - roll
	// motor 5 = total pitch + roll
	delta.r[4] = AngleToSteps(4, dt2 + dt3 + dt4 - dt5);
	delta.r[5] = AngleToSteps(5, dt2 + dt3 + dt4 + dt5);

	// Gripper follows its own change only
	delta.r[6] = mmToStepsGrip(dg);
	delta.r[7] = 0;

	std::cout << "\nMoveTo delta steps:\n";
	std::cout << "M1: " << delta.r[1] << "\n";
	std::cout << "M2: " << delta.r[2] << "\n";
	std::cout << "M3: " << delta.r[3] << "\n";
	std::cout << "M4: " << delta.r[4] << "\n";
	std::cout << "M5: " << delta.r[5] << "\n";
	std::cout << "M6: " << delta.r[6] << "\n";

	int out = SendStep(speed, delta);
	if (out != 1)
	{
		std::cout << "SendStep failed with code " << out << "\n";
		return 0;
	}

	// Reconstruct achieved joint motion from ACTUAL commanded steps
	Jointspace achievedJoint = currentJoint;

	double d1 = StepsToAngle(1, delta.r[1]);
	double d2 = StepsToAngle(2, delta.r[2]);

	// Motor 3 represents d2 + d3
	double d23 = StepsToAngle(3, delta.r[3]);
	double d3 = d23 - d2;

	// Motor 4 = d2 + d3 + d4 - d5
	// Motor 5 = d2 + d3 + d4 + d5
	double a45 = StepsToAngle(4, delta.r[4]);
	double b45 = StepsToAngle(5, delta.r[5]);

	double d5 = 0.5 * (b45 - a45);
	double d4 = 0.5 * (a45 + b45) - d23;

	achievedJoint.t[0] += d1;
	achievedJoint.t[1] += d2;
	achievedJoint.t[2] += d3;
	achievedJoint.t[3] += d4;
	achievedJoint.t[4] += d5;

	// Build achieved task pose from FK
	if (!ForwardKinematics(achievedJoint, achievedTask))
	{
		std::cout << "MoveTo ERROR: FK failed after move.\n";
		return 0;
	}

	achievedTask.g = lastTask.g + stepsToMmGrip(delta.r[6]);

	std::cout << "\nRequested target:\n";
	std::cout << "x=" << t.x
		<< " y=" << t.y
		<< " z=" << t.z
		<< " p=" << RadtoDeg(t.p)
		<< " r=" << RadtoDeg(t.r)
		<< " g=" << t.g << "\n";

	std::cout << "FK estimated achieved pose:\n";
	std::cout << "x=" << achievedTask.x
		<< " y=" << achievedTask.y
		<< " z=" << achievedTask.z
		<< " p=" << RadtoDeg(achievedTask.p)
		<< " r=" << RadtoDeg(achievedTask.r)
		<< " g=" << achievedTask.g << "\n";

	std::cout << "Error:\n";
	std::cout << "dx=" << (achievedTask.x - t.x)
		<< " dy=" << (achievedTask.y - t.y)
		<< " dz=" << (achievedTask.z - t.z)
		<< " dp=" << RadtoDeg(achievedTask.p - t.p)
		<< " dr=" << RadtoDeg(achievedTask.r - t.r)
		<< " dg=" << (achievedTask.g - t.g) << "\n";

	// Update remembered pose to what was actually achieved
	lastTask = achievedTask;

	// Return achieved pose to caller
	t = achievedTask;

	return 1;
}


int Microbot::GoHome(int speed)
{
	Taskspace target = homePos;

	std::cout << "\n--- Moving to HOME position ---\n";

	return MoveTo(target, speed);
}

int Microbot::ResetHome()
{
	lastTask = homePos;
	std::cout << "Software reset to HOME position.\n";
	return 1;
}

int Microbot::PrintCurrentPosition()
{
	std::cout << "\n--- Current Robot Position ---\n";

	std::cout << "x: " << lastTask.x << " mm\n";
	std::cout << "y: " << lastTask.y << " mm\n";
	std::cout << "z: " << lastTask.z << " mm\n";

	std::cout << "p: " << RadtoDeg(lastTask.p) << " deg\n";
	std::cout << "r: " << RadtoDeg(lastTask.r) << " deg\n";

	std::cout << "g: " << lastTask.g << " mm\n";

	return 1;
}
