#include "kinematics.h"
#include <iostream>
#include <limits>

void clearInput()
{
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

void printMenu()
{
	std::cout << "\n=== Microbot Menu ===\n";
	std::cout << "1. Send Manual Step Command\n";
	std::cout << "2. Send IK Command\n";
	std::cout << "3. Set Home Posistion\n";
	std::cout << "4. Go To Home Posisiton\n";
	std::cout << "5. Exit\n";
	std::cout << "Choose an option: ";
}

int main()
{
    Microbot robot;
    Registerspace delta;
    Taskspace t;

    int speed = 235;
    int choice;

    bool running = true;
    while (running)
    {
        printMenu();
        std::cin >> choice;

        if (std::cin.fail())
        {
            clearInput();
            std::cout << "Invalid menu input.\n";
            continue;
        }

        switch (choice)
        {
        case 1:
        {
            std::cout << "Enter speed (0-240): ";
            std::cin >> speed;

            if (std::cin.fail())
            {
                clearInput();
                std::cout << "Invalid speed.\n";
                break;
            }

            if (speed > 240) speed = 240;
            if (speed < 0) speed = 0;

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

        case 2:
        {
            std::cout << "Enter speed (0-240): ";
            std::cin >> speed;

            if (std::cin.fail())
            {
                clearInput();
                std::cout << "Invalid speed.\n";
                break;
            }

            if (speed > 240) speed = 240;
            if (speed < 0) speed = 0;

            std::cout << "\nEnter target task-space values:\n";
            std::cout << "x y z p r g: ";
            std::cin >> t.x >> t.y >> t.z >> t.p >> t.r >> t.g;

            if (std::cin.fail())
            {
                clearInput();
                std::cout << "Invalid IK input.\n";
                break;
            }

            if (!robot.MoveTo(t, speed))
            {
                std::cout << "Move failed.\n";
            }

            break;
        }

        case 3:
        {
            std::cout << "section not yet ready\n";
            break;
        }

        case 4:
        {
            std::cout << "section not yet ready\n";
            break;
        }

        case 5:
        {
            running = false;
            std::cout << "Exiting program.\n";
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
