#include "kinematics.h"

int main()
{
	Microbot robot;				// Local variable of the microbot class
	Registerspace delta;		// Local variable for input of motor steps
	Jointspace j;				// Local variable for kinematic calculations
	Taskspace t;				// Local variable for kinematic calculations

	int spe = 235;				// Motor speed; should not be higher than 240
	int i = 1;
	int out = 0;

// Example; replace it with your own program

	delta.r[7] = 0;				// Assign number of steps for each motor
	delta.r[6] = 0;
	delta.r[5] = 0;
	delta.r[4] = 0;
	delta.r[3] = 0;
	delta.r[2] = 0;
	delta.r[1] = -200;

	while(i<6) {
		out = robot.SendStep(spe, delta);	// Send instruction to the microbot
		printf("i= %d, out= %d\n", i, out);
		fflush(stdout);
		i++;
	};

	printf("Enter 1000 for delta: ");fflush(stdout);
	scanf("%d", &delta.r[1]);

	robot.SendStep(spe, delta);

	t.x = 20; // Taskspace data to be passed in by value

	printf("Input to InverseKinematics: t.x= %f\n", t.x); // Shows how to display a structured variable
	fflush(stdout);

	out = robot.InverseKinematics(t,j); // Jointspace data retrieved by reference

	printf("Output from InverseKinematics: j.t[1]= %f, return= %d\n", j.t[1], out);
	fflush(stdout);

	delta.r[1] = BASE_STEPS * j.t[1];

	printf("Done, hit Enter to exit: ");fflush(stdout);
	getchar();
	printf("Good bye.\n");
}
