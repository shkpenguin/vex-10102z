#include "controller.h"
#include "robot-config.h"

int t, A1, A2, A3, A4, L1, L2, R1, R2, X, Y, A, B, LEFT, RIGHT, UP, DOWN,
            last_L1, last_L2, last_R1, last_R2, 
            last_X, last_Y, last_A, last_B, last_LEFT, last_RIGHT, last_UP, last_DOWN;

bool press_X, press_Y, press_A, press_B, press_UP, press_DOWN, press_LEFT, press_RIGHT;
bool release_L2, press_L2;
bool isUsrCtl, last_isUsrCtl; 

void defineController(){
    while (true)
    {
        last_L1 = L1;
        last_L2 = L2;
        last_R1 = R1;
        last_R2 = R2;
        last_X = X;
        last_Y = Y;
        last_A = A;
        last_B = B;
        last_LEFT = LEFT;
        last_RIGHT = RIGHT;
        last_UP = UP;
        last_DOWN = DOWN;
        t = Brain.timer(vex::timeUnits::msec);
        A1 = Controller1.Axis1.position(vex::percentUnits::pct);
        A2 = Controller1.Axis2.position(vex::percentUnits::pct);
        A3 = Controller1.Axis3.position(vex::percentUnits::pct);
        A4 = Controller1.Axis4.position(vex::percentUnits::pct);
        L1 = Controller1.ButtonL1.pressing();
        L2 = Controller1.ButtonL2.pressing();
        R1 = Controller1.ButtonR1.pressing();
        R2 = Controller1.ButtonR2.pressing();
        X = Controller1.ButtonX.pressing();
        Y = Controller1.ButtonY.pressing();
        A = Controller1.ButtonA.pressing();
        B = Controller1.ButtonB.pressing();
        LEFT = Controller1.ButtonLeft.pressing();
        RIGHT = Controller1.ButtonRight.pressing();
        UP = Controller1.ButtonUp.pressing();
        DOWN = Controller1.ButtonDown.pressing();

        // pressFlag
        if (X && !last_X) press_X = true;
        if (A && !last_A) press_A = true;
        if (B && !last_B) press_B = true;
        if (Y && !last_Y) press_Y = true;
        if (UP && !last_UP) press_UP = true;
        if (DOWN && !last_DOWN) press_DOWN = true;
        if (RIGHT && !last_RIGHT) press_RIGHT = true;
        if (LEFT && !last_LEFT) press_LEFT = true;
        if (L2 && !last_L2) press_L2 = true;
        if (last_L2 && !L2) release_L2 = true;
        // std::cout<<DOWN<<' '<<last_DOWN<<std::endl;

        this_thread::sleep_for(10);
    }
}

