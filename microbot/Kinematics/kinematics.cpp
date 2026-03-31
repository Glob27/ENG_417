#include "kinematics.h"
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cstdio>

Taskspace homePos = { 125, 0, 20, (90 * (PI / 180)), 0, 0 };
Taskspace lastTask = homePos;

double RadtoDeg(double rad)
{
	return rad * (180.0 / PI);
}

int Microbot::InverseKinematics(Taskspace t, Jointspace& j)
{
	printf("inside InverseKinematics\n");
	fflush(stdout);

	double px = t.x;
	double py = t.y;
	double pz = t.z;
	double p = t.p;
	double r = t.r;

	double theta1 = std::atan2(py, px);
	double theta234 = p + (PI / 2.0);
	double theta5 = r;

	double c1 = std::cos(theta1);
	double s1 = std::sin(theta1);
	double c234 = std::cos(theta234);
	double s234 = std::sin(theta234);

	// wrist center
	double Wx = px - d * c1 * s234;
	double Wy = py - d * s1 * s234;
	double Wz = pz + d * c234;

	// planar coordinates for shoulder/elbow problem
	double R = std::sqrt(Wx * Wx + Wy * Wy); // horizontal reach from base axis
	double Z = Wz - h;                       // vertical height from shoulder plane

	// elbow cosine law
	double c3_raw = (R * R + Z * Z - a * a - a * a) / (2.0 * a * a);

	if (c3_raw > 1.0 || c3_raw < -1.0) {
		std::cout << "IK ERROR: Position out of reach\n";
		return 0;
	}

	double c3 = std::max(-1.0, std::min(1.0, c3_raw));

	// pick elbow-down branch
	double s3 = -std::sqrt(1.0 - c3 * c3);
	double theta3 = std::atan2(s3, c3);

	// standard 2-link solution
	double k1 = a + a * c3;
	double k2 = a * s3;

	double theta2 = std::atan2(Z, R) - std::atan2(k2, k1);

	double theta4 = theta234 - theta2 - theta3;

	j.t[0] = theta1;
	j.t[1] = theta2;
	j.t[2] = theta3;
	j.t[3] = theta4;
	j.t[4] = theta5;

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
		1125.0, // Motor 1 (Base)
		1125.0, // Motor 2 (Shoulder)
		672.0,  // Motor 3 (Elbow)
		244.4,  // Motor 4 (Wrist differential side A)
		244.4,  // Motor 5 (Wrist differential side B)
		244.4   // Motor 6 (Gripper placeholder)
	};

	// Software-only direction correction
	// Flip these signs if any axis still moves opposite
	static const int motorSign[] = {
		0,   // unused
		1,   // Motor 1
		-1,  // Motor 2
		-1,  // Motor 3
		1,   // Motor 4
		1,   // Motor 5
		1    // Motor 6
	};

	return static_cast<int>(motorSign[motor] * angleRad * stepsPerRad[motor - 1]);
}

int mmToStepsGrip(double grip)
{
	return static_cast<int>(grip * 13.4);
}

int Microbot::MoveTo(Taskspace& t, int speed)
{
	Jointspace currentJoint, targetJoint;
	Registerspace delta = {};

	// Find current joint values from remembered task pose
	if (!InverseKinematics(lastTask, currentJoint))
	{
		std::cout << "MoveTo ERROR: could not solve IK for starting position.\n";
		return 0;
	}

	// Find target joint values
	if (!InverseKinematics(t, targetJoint))
	{
		std::cout << "MoveTo ERROR: could not solve IK for target position.\n";
		return 0;
	}

	// Base / shoulder / elbow
	delta.r[1] = AngleToSteps(1, targetJoint.t[0] - currentJoint.t[0]);
	delta.r[2] = AngleToSteps(2, targetJoint.t[1] - currentJoint.t[1]);
	delta.r[3] = AngleToSteps(3, targetJoint.t[2] - currentJoint.t[2]);

	// Wrist differential:
	// same direction  -> pitch
	// opposite direction -> roll
	//
	// So:
	// motor4 = pitch + roll
	// motor5 = pitch - roll
	//
	// Here:
	// j.t[3] = wrist pitch
	// j.t[4] = wrist roll

	double currentPitch = currentJoint.t[3];
	double currentRoll = currentJoint.t[4];

	double targetPitch = targetJoint.t[3];
	double targetRoll = targetJoint.t[4];

	double deltaPitch = targetPitch - currentPitch;
	double deltaRoll = targetRoll - currentRoll;

	double motor4Delta = deltaPitch + deltaRoll;
	double motor5Delta = deltaPitch - deltaRoll;

	delta.r[4] = AngleToSteps(4, motor4Delta);
	delta.r[5] = AngleToSteps(5, motor5Delta);

	// Gripper should use change, not absolute target
	delta.r[6] = mmToStepsGrip(t.g - lastTask.g);
	delta.r[7] = 0;

	// Debug output
	std::cout << "MoveTo delta steps:\n";
	std::cout << "M1: " << delta.r[1] << "\n";
	std::cout << "M2: " << delta.r[2] << "\n";
	std::cout << "M3: " << delta.r[3] << "\n";
	std::cout << "M4: " << delta.r[4] << "\n";
	std::cout << "M5: " << delta.r[5] << "\n";
	std::cout << "M6: " << delta.r[6] << "\n";

	SendStep(speed, delta);

	// Remember new commanded task pose
	lastTask = t;
	return 1;
}