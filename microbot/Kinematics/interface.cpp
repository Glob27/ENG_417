#include "kinematics.h"

constexpr unsigned int NUM_MOTORS=7; //number of motors
constexpr unsigned int MAXTIME=1000000; //timer

using String=std::string;   //for command string to send to microbot

Microbot::Microbot(){
	port.Open(1,9600);
};

int Microbot::SendStep(int speed, Registerspace del)
{
	int ret_num = 0; //returned from readFile()
	double timer = MAXTIME;
	char ret[2];
	/*
	 * create a C++ String to hold the instruction for the microbot
	 */
	String step_cmd;
	step_cmd.append("@STE "); //step command
	//populate the string with SendStep arguments
	//first element is the speed
	step_cmd.append(std::to_string(speed));
	// seperate each element with a comma
	step_cmd.append(",");
	//populate string with the motor positions from Registerspace
	for(unsigned int i=1; i <= NUM_MOTORS; i++){
		step_cmd.append(std::to_string(del.r[i]));
		step_cmd.append(",");
	}
	// put a carriage return "\r" at the end of command
	step_cmd.replace(step_cmd.end()-1,step_cmd.end(),"\r");

	// print statement for debugging; may be commented out afterwards
	//printf("String = %s\n", step_cmd.c_str());
	fflush( stdin );
	//send the ascii string to microbot
	port.SendData(step_cmd.c_str(), step_cmd.length());
	//std::cout  << "sending " << step_cmd << " of length " << step_cmd.length() << "\n";

	while ((ret_num == 0) && (timer > 0))
		{
			ret_num = port.ReadData(ret, 1);
			timer = (timer - 0.25);
		}

	char cmd_status = ret[0];
	ret_num = 0;
	timer = 1000000;
	while ((ret_num == 0) && (timer > 0))
	{
		ret_num = port.ReadData(ret, 1);
		timer = (timer - 0.25);
	}

	ret[1] = ret[0];
	ret[0] = cmd_status;
	if(timer <= 0)
		printf("Error");
	
	int i;
	if(ret_num!=0)
		{
		// print statement for debugging; may be commented out afterwards
		//printf( "Return from Microbot: %c\n", ret[0]);
		ret[1] = '\0';
		i = atoi(ret);
		}
	
	return i;
	
}


int Microbot::SendClose(int speed, int force)
{

	int i;
	char ret[2];
	//char spe[5];
	char c[80] = "@CLO ";

	Registerspace r;

	std::string spe=std::to_string(speed);
	//_itoa(speed, spe, 10);
	

	if (speed != -1)
	{
	strcat(c, spe.c_str());
	
	}

	strcat(c, "\r");

	printf("String = %s\n",c);

	fflush( stdin );

	i = strlen(c);

	port.SendData(c,i);

	i = 0;

	while(i==0)
		i = port.ReadData(ret,2);

	i = 1;

if (force != -1)
	{

	if (speed == -1)
	{speed = 221;}

	r.r[1] = 0;
	r.r[2] = 0;
	r.r[3] = 0;
	r.r[4] = 0;
	r.r[5] = 0;
	r.r[6] = -10*force;

	i = SendStep( speed, r );
	}

return i;		

}



int Microbot::SendRead(Registerspace *read)
{
	int	i, count=1;
	int a=0;
	int ret_num=0;


	char c[10] = "@READ\r";
	char d[80];

	i = strlen(c);
	port.SendData(c,i);

	while(ret_num==0)
		{			
			
			ret_num = port.ReadData(d,1);
	
		}
	

	while(a<2)
	{
		ret_num = 0;

		while(ret_num==0)
			ret_num = port.ReadData(d+count,1);
		
		if(d[count]=='\r')
			a++;

		count++;
	
	}
	
	d[count-1] = '\0';

	int index=2;
	int index1=1;

	while(d[index]!='\0')
	{

	read->r[index1]	 = atoi(d+index);

	index1++;

		while(d[index]!=','&&d[index]!='\0')
			index++;

		if(d[index]==',')
			index++;
	}

	printf("Motor steps in ascii =  %s\n", d+2);
	
	

	return 0;
}


int Microbot::SendSet(int speed)
{	
	int	i;
	int ret_num=0;
	char ret[2];
//, spe[5];
	char c[15] = "@SET";
	
	std::string spe=std::to_string(speed);
	//_itoa(speed, spe, 10);
	
	strcat(c, spe.c_str());
	strcat(c, ",");
	strcat(c, "\r");
	
	printf("String = %s\n",c);
	
	i = strlen(c);
	port.SendData(c,i);
	
	while(ret_num==0)
		ret_num = port.ReadData(ret,2);

	ret[1] = '\0';
	i = atoi(ret);
	
	return i;
}

int Microbot::SendReset()
{

	int i;
	//int ret_num=0;
	char ret[2];
	char c[80] = "@RESET ";

	
	strcat(c, "\r");

	printf("String = %s\n",c);

	fflush( stdin );

	i = strlen(c);

	port.SendData(c,i);

	i = 0;

	while(i==0)
		i = port.ReadData(ret,2);

	i = 1;
return i;		

}
