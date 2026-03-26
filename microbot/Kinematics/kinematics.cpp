#include "kinematics.h"
#include <array>
#include <cmath>

int Microbot::InverseKinematics(Taskspace t, Jointspace &j){

	int i = 0;

    printf("inside InverseKinematics\n");
    fflush(stdout);

    // Extract taskspace values
    double px = t.x;
    double py = t.y;
    double pz = t.z;
    double p  = t.p;
    double r  = t.r;
	
	double theta1 = std::atan2(t.y, t.x);
	double theta234 = t.p + (PI / 2); //usig the radain version of 90 degrees as all function in C++ use rads
	double theta5 = t.r;
	
	//trig we can do so far
	double c1 = cos(theta1);
	double s1 = sin(theta1);
	double c234 = cos(theta234);
	double s234 = sin(theta234);
	double c5 = cos(theta5);
	double s5 = sin(theta5);


	//ok getting just theta2 is a bit of a proscess becouse i got to get wrist pos values
	double Wx = t.x - d * c1 * s234;
	double Wy = t.y - d * s1 * s234;
	double Wz = t.z + d * c234;

	//we can now get theta3
	double c3 = (Wx^2 + Wy^2 + (Wz - h) ^ 2) / (2 * a^2);
	double s3 = -*sqrt(1 - c3^2); //(theta3 < 0)
	double theta3 = std::atan2(s3, c3);
	
	//now on to theta2
	double c2 = ((Wz - h) * s3 + sqrt(Wx^2 + Wy^2) * (1 + c3)) / (2a(1 + c3));
	double s2 = ((Wz - h) * (1 + c3) - s3 * sqrt(Wx ^ 2 + Wy ^ 2)) / (2a(1 + c3));
	double theta2 = std::atan2(s2, c2);

	double theta4 = theta234 - theta2 - theta3;

	//now we put em in a matrix
	j.t[0] = theta1;
	j.t[1] = theta2;
	j.t[2] = theta3;
	j.t[3] = theta4;
	j.t[4] = theta5;


	printf("inside InverseKinematics\n");  // to be removed when the function is complete //ill remove this after testing
	fflush(stdout); //assuming this needs to be removed too

	

	i = 1; // assign a return value, usually error code
	return(i);
}

int Microbot::ForwardKinematics(Jointspace j, Taskspace &t){
	//Were starting by defining the matrix for the Robot
	using ForwardKinematicsMatrix = std::array<std::array<double, 4>, 4>;


	



	return(0);
}

int Microbot::MoveTo(Taskspace &t){
	// write your move-to function here
	return(0);
}
