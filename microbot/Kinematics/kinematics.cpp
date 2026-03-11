#include "kinematics.h"


int Microbot::InverseKinematics(Taskspace t, Jointspace &j){

	// write your inverse kinematics here, here is a sample:

	int i = 0; // declaration for the return variable

	printf("inside InverseKinematics\n");  // to be removed when the function is complete
	fflush(stdout);

	j.t[1] = t.x + PI; // use passed-through data (t) to calculate return data (j)

	i = 1; // assign a return value, usually error code
	return(i);
}

int Microbot::ForwardKinematics(Jointspace j, Taskspace &t){
	// write your forward kinematics here
	return(0);
}

int Microbot::MoveTo(Taskspace &t){
	// write your move-to function here
	return(0);
}
