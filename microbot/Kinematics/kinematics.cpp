#include "kinematics.h"
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>



Taskspace homePos = { 125, 0, 10, (15 * (PI / 180)), 0, 1 };
Taskspace lastTask = homePos;

double RadtoDeg(double rad)
{
	return rad * (180 / PI);
}

int Microbot::InverseKinematics(Taskspace t, Jointspace& j){

	int i = 0;

    printf("inside InverseKinematics\n");
    fflush(stdout);

    // Extract taskspace values
    double px = t.x;
    double py = t.y;
    double pz = t.z;
    double p  = t.p;
    double r  = t.r;
	
	double theta1 = std::atan2(py, px);
	double theta234 = p + (PI / 2); //usig the radain version of 90 degrees as all function in C++ use rads
	double theta5 = r;
	
	//trig we can do so far
	double c1 = cos(theta1);
	double s1 = sin(theta1);
	double c234 = cos(theta234);
	double s234 = sin(theta234);


	//ok getting just theta2 is a bit of a proscess becouse i got to get wrist pos values
	double Wx = px - d * c1 * s234;
	double Wy = py - d * s1 * s234;
	double Wz = pz + d * c234;

	//we can now get theta3
	double c3_raw = ((Wx * Wx) + (Wy * Wy) + ((Wz - h) * (Wz - h))) / (2 * (a * a)) - 1;
	
	//making sure we arent outside the work space
	if (c3_raw > 1.0 || c3_raw < -1.0) {
		std::cout << "IK ERROR: Position out of reach\n";
		return 0;  // or some failure flag
	}

	// Now safe to clamp small numerical errors
	double c3 = std::max(-1.0, std::min(1.0, c3_raw));
	
	if (fabs(1 + c3) < 1e-6) {
		std::cout << "IK WARNING: singular configuration\n"; //prevents dividing by zero
		return 0;
	}


	double s3 = -sqrt(1 - (c3*c3)); //(theta3 < 0)
	double theta3 = std::atan2(s3, c3);
	
	//now on to theta2
	double c2 = ((Wz - h) * s3 + sqrt((Wx * Wx) + (Wy * Wy)) * (1 + c3)) / (2*a*(1 + c3));
	double s2 = ((Wz - h) * (1 + c3) - s3 * sqrt((Wx * Wx) + (Wy * Wy))) / (2*a*(1 + c3));
	double theta2 = std::atan2(s2, c2);

	double theta4 = theta234 - theta2 - theta3;

	//now we put em in a matrix
	j.t[0] = theta1;
	j.t[1] = theta2;
	j.t[2] = theta3;
	j.t[3] = theta4;
	j.t[4] = theta5;

	theta1 = RadtoDeg(theta1);
	theta2 = RadtoDeg(theta2);
	theta3 = RadtoDeg(theta3);
	theta4 = RadtoDeg(theta4);
	theta5 = RadtoDeg(theta5);

	printf("angles the microbot is using %d %d %d %d %d \n", theta1, theta2, theta3, theta4, theta5);
	//printf("inside InverseKinematics\n");  // to be removed when the function is complete //ill remove this after testing
	fflush(stdout); //assuming this needs to be removed too

	

	i = 1; // assign a return value, usually error code
	return(i);
}

int Microbot::ForwardKinematics(Jointspace j, Taskspace &t){
	// Forward kinematics implemented using closed-form expressions
	// derived from T_5^0 (see HW #3), without explicitly constructing the matrix

	//JOINT ANGLES
	double theta1 = j.t[0];
	double theta2 = j.t[1];
	double theta3 = j.t[2];
	double theta4 = j.t[3];
	double theta5 = j.t[4];
	
	//Combanation angles
	double theta23 = theta2 + theta3;
	double theta234 = theta2 + theta3 + theta4;
	
	//trig functions
	double c1 = cos(theta1);
	double s1 = sin(theta1);

	double c2 = cos(theta2);
	double s2 = sin(theta2);

	double c23 = cos(theta23);
	double s23 = sin(theta23);

	double c234 = cos(theta234);
	double s234 = sin(theta234);

	//the final values based on the 
	t.x = c1 * (a * c2 + a * c23 + d * s234);
	t.y = s1 * (a * c2 + a * c23 + d * s234);
	t.z = h + a * s2 + a * s23 - d * c234;

	t.p = theta234 - (PI / 2);
	t.r = theta5;

	return 1;
}

int AngleToSteps(int motor, double angleRad) {
	static const double stepsPerRad[] = {
		1125,   // Motor 1 (Base)
		1125,   // Motor 2 (Shoulder)
		672,    // Motor 3 (Elbow)
		244.4,  // Motor 4 (Right wrist)
		244.4,	//Motor 5 (Left wrist)
		244.4	//Motr 6 (Gripper) //Not the accual ration we need to find this one
	};

	return static_cast<int>(angleRad * stepsPerRad[motor - 1]);
}

int Microbot::MoveTo(Taskspace &t, int speed){
	Jointspace currentJoint, targetJoint;
	Registerspace delta;

	// Convert previous task position to joint angles
	if (!InverseKinematics(lastTask, currentJoint))
	{
		std::cout << "MoveTo ERROR: could not solve IK for starting position.\n";
		return 0;
	}

	// Convert target task position to joint angles
	if (!InverseKinematics(t, targetJoint))
	{
		std::cout << "MoveTo ERROR: could not solve IK for target position.\n";
		return 0;
	}

	// Convert angle differences to step differences
	delta.r[1] = AngleToSteps(1, targetJoint.t[0] - currentJoint.t[0]);
	delta.r[2] = AngleToSteps(2, targetJoint.t[1] - currentJoint.t[1]);
	delta.r[3] = AngleToSteps(3, targetJoint.t[2] - currentJoint.t[2]);
	delta.r[4] = AngleToSteps(4, targetJoint.t[3] - currentJoint.t[3]);
	delta.r[5] = AngleToSteps(5, targetJoint.t[4] - currentJoint.t[4]);
	delta.r[6] = 0; // gripper not being moved here
	delta.r[7] = 0; // keep unused slot zero

	SendStep(speed, delta);

	lastTask = t;
	return 1;
}
