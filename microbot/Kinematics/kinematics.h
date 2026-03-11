#ifndef KINEMATICS_H
#define KINEMATICS_H


/*
	ELE 317/517 - Introduction to Robotics.
	Header file containing definitions of the Microbot class.
	This class contains data members and functions dealing
	with the Microbot.

	Last revision date : 2-4-99 by Jamie Stultz
	Last revision date : 10-11-2018 by Scott Harding: Replaced all Pointer function parms with type Reference
		so function args do not need to be passed using address-of "&" operator, e.g., InverseKinematics(t,&j) becomes
		InverseKinematics(t,j).
	Last revision date : 6-18-2019 by Scott Harding: simplify the headers for exclusive use by EGN 317

*/
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
//#include <iostream>
#include "serial.h"

#define BASE_STEPS 7072
#define SHOULDER_STEPS 7072
#define ELBOW_STEPS 4158
#define RIGHT_STEPS 1536
#define LEFT_STEPS 1536
#define GRIPPER_STEPS 375

constexpr double PI = acos(-1);

// Public Data Structures
	struct Taskspace
	{
		double x,y,z,p,r,g;
	};

	struct Jointspace
	{
		double t[7];
	};

	struct Registerspace
	{
		int r[9];
	};

class Microbot
{
private:
	CSerial port;

public:

// Constructor
	Microbot();

// Public Member Functions
	// to be written in kinematics.cpp
	int InverseKinematics(Taskspace t, Jointspace &j);
	int ForwardKinematics(Jointspace j, Taskspace &t);
	int MoveTo(Taskspace &t);
	
	// written in interface.cpp
	int SendStep(int speed, Registerspace delta);
	int SendClose(int speed, int force);
	int SendRead(Registerspace *read);
	int SendSet(int speed);
    int SendReset();

};
#endif
