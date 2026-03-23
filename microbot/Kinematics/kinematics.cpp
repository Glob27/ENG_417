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
	double theta5 = t.r
	
	//ok getting just theta2 is a bit of a proscess becouse i got to get wrist pos values
	




	int i = 0; // declaration for the return variable

	printf("inside InverseKinematics\n");  // to be removed when the function is complete
	fflush(stdout);

	j.t[1] = t.x + PI; // use passed-through data (t) to calculate return data (j)

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
