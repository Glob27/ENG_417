#include "kinematics.h"
#include <iostream>
#include <limits>

void clearInput()
{
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

//this prints the menu that the user sees
void printMenu()
{
    std::cout << "\n=== Microbot Menu ===\n";
    std::cout << "1. Send Manual Step Command\n";
    std::cout << "2. Send IK Command\n";
    std::cout << "3. Current Position Data\n";
    std::cout << "4. Go To Home Position\n";
    std::cout << "5. Exit\n";
    std::cout << "6. Reset Home\n";
    std::cout << "7. Straight Line\n";
    std::cout << "Choose an option: ";
}

//used to convert user degree input into program rad
double degToRad(double deg)
{
    return deg * (PI / 180.0);
}

//the main function
int main()
{
    //restering stuff 
    Microbot robot;
    Registerspace delta;
    Taskspace t;

    //defining speed and choice
    int speed = 235;
    int choice = 0;

    bool running = true;
    while (running) //making the while function that the user interacts with
    {
        printMenu(); //printing the menu
        std::cin >> choice; //userinput

        if (std::cin.fail()) //checks if an input failed, example, they tried using letters instead of numbers //not 100 percent on this one it was a while ago 
            //and prevent users from breaking stuff is something that confuses me
        {
            clearInput();
            std::cout << "Invalid menu input.\n";
            continue;
        }

        switch (choice) //the choice is sent through the switch function 
        {
        case 1: //case one the user chooses a speed, then speeds direct step commands //hold over from early testing
        {
            std::cout << "Enter speed (200-240): "; //speed choice
            std::cin >> speed;

            if (std::cin.fail())
            {
                clearInput();
                std::cout << "Invalid speed.\n";
                break;
            }

            if (speed > 240) speed = 240;
            if (speed < 200) speed = 200; //changed this from 0 to 200 since 0 isnt usefull 

            std::cout << "\nEnter 7 motor steps (m1 m2 m3 m4 m5 m6 m7):\n";
            std::cin >> delta.r[1] >> delta.r[2] >> delta.r[3]
                >> delta.r[4] >> delta.r[5] >> delta.r[6] >> delta.r[7];

            if (std::cin.fail())
            {
                clearInput();
                std::cout << "Invalid motor input.\n";
                break;
            }

            robot.SendStep(speed, delta);
            break;
        }

        case 2: //this is the function where we send the EE pos that we want the robot to use
        {
            std::cout << "Enter speed (200-240): "; //first the speed 
            std::cin >> speed;

            if (std::cin.fail())
            {
                clearInput();
                std::cout << "Invalid speed.\n";
                break;
            }

            if (speed > 240) speed = 240;
            if (speed < 200) speed = 200; //I choose the 200 becouse the microbot tends to not peroform well at speeds slower than this

            std::cout << "\nEnter target task-space values:\n";
            std::cout << "x y z p r g: "; //we enter target values
            double p_deg, r_deg;

            std::cin >> t.x >> t.y >> t.z >> p_deg >> r_deg >> t.g;

            t.p = degToRad(p_deg);
            t.r = degToRad(r_deg);

            if (std::cin.fail())
            {
                clearInput();
                std::cout << "Invalid IK input.\n";
                break;
            }

            if (!robot.MoveTo(t, speed)) //we send the target values to our move to function
            {
                std::cout << "Move failed.\n";
            }

            break;
        }

        case 3: //just prints the current pos
        {
            robot.PrintCurrentPosition();
            break;
        }

        case 4: //sends the robot home
        {
            std::cout << "Enter speed (200-240): ";
            std::cin >> speed;

            if (std::cin.fail())
            {
                clearInput();
                std::cout << "Invalid speed.\n";
                break;
            }

            if (speed > 240) speed = 240;
            if (speed < 200) speed = 200; //also changed this too 200

            robot.GoHome(speed);
            break;
        }

        case 5: //ends the program
        {
            running = false;
            std::cout << "Exiting program.\n";
            break;
        }
        case 6: //resets to the home position
        {
            robot.ResetHome();
            break;
        }
        case 7:
        {
            std::cout << "Straight Line Choosen:\n"; //first the speed 
            std::cout << "Enter speed (200-240): "; //first the speed 
            std::cin >> speed;

            if (std::cin.fail())
            {
                clearInput();
                std::cout << "Invalid speed.\n";
                break;
            }

            if (speed > 240) {
                speed = 240;
            }
            if (speed < 200) {
                speed = 200;
            } //I choose the 200 becouse the microbot tends to not peroform well at speeds slower than this

            bool repeating = true;

            while (repeating) {
                std::cout << "\nEnter target task-space values:\n";
                std::cout << "x y z p r g: "; //we enter target values
                double p_deg, r_deg;

                std::cin >> t.x >> t.y >> t.z >> p_deg >> r_deg >> t.g;

                t.p = degToRad(p_deg);
                t.r = degToRad(r_deg);

                if (std::cin.fail())
                {
                    clearInput();
                    std::cout << "Invalid IK input.\n";
                    break;
                }

                if (!robot.linePlotting(t, speed)) //we send the target values to our move to function
                {
                    std::cout << "Move failed.\n";
                }
                std::cout << "Repeat? Y/N";
                std::cin >> option;

                if (option != "y" || option != "Y") {
                    repeating = false;
                }
            }
            break;
        }

        default:
        {
            std::cout << "Invalid option.\n";
            break;
        }
        }
    }

    return 0;
}