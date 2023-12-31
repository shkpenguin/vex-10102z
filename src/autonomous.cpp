#include "autonomous.h"
#include "vex.h"
#include "usercontrol.h"
#include "auto-function.h"
#include "utils.h"
#include "robot-config.h"
#include "timer.h"
using namespace vex;

void autonomous() {

    setIMUHeading(315);
    quickMoveDistance(-60, 100, 1300);
    quickMoveDistance(12, 100, 1800);
    quickTurnTo(73, 100);
    quickMoveDistance(4,100,500);
    Timer autoTimer;
    double currentHeading = IMUHeading();
    catapult.spin(fwd, motorVoltage * 100, voltageUnits::mV);    
    Brain.Screen.print("IMUHeading: %.1f \n", currentHeading);
    while(true) 
    {        
        if (autoTimer.getTime() > 33000 ) {
            setIMUHeading(currentHeading);
            catapult.stop();
            break;
        }
    }

    quickTurnTo(145, 100);
    quickMoveDistance(-25, 100, 1000);
    quickTurnTo(90, 100);
    Brain.Screen.print("IMUHeading: %.1f \n", IMUHeading());
    quickMoveDistance(-95, 100, 2100);
    quickTurnTo(30, 100);
    openWings();
    quickMoveDistance(-30, 100, 1300);
    closeWings();
    quickMoveDistance(20, 100, 800);
    quickMoveDistance(-60, 100, 1000);
    quickMoveDistance(12, 100, 800);

    quickTurnTo(110, 100);
    quickMoveDistance(46, 100, 1300);
    openWings();
    quickTurnTo(60,100);
    quickMoveDistance(-100, 100, 1800);
    closeWings();
    // second stage
    //quickTurnTo(90, 100);  // 120 -145
    quickMoveDistance(35, 100, 1200);
    quickTurnTo(180, 100);
    quickMoveDistance(15, 100, 1200);
    quickTurnTo(90, 100);
    openWings();
    quickMoveDistance(-100, 100, 1000);
    closeWings();

    quickMoveDistance(35, 100, 1200);
    quickTurnTo(180, 100);
    quickMoveDistance(25, 100, 1200);
    quickTurnTo(110, 100);
    openWings();
    quickMoveDistance(-100, 100, 1000);
    closeWings();


    // quickTurnTo(80, 100);    
    // quickMoveDistance(-100, 100, 1000);
    // quickMoveDistance(25, 100, 800);
    // quickTurnTo(110, 100);
    

    // third stage
    quickMoveDistance(5, 100, 300);
    quickTurnTo(180, 100);
    quickMoveDistance(50, 100, 1200);
    openWings();
    quickTurnTo(135, 100);
    quickMoveDistance(-100, 100, 1000);
    closeWings();
    quickMoveDistance(15, 100, 1300);
    quickMoveDistance(-100, 100, 1000);

 
    // further side:
    /*
    intakeTriball(100);
    quickMoveDistance(3, 100, 500);
    quickMoveDistance(-33, 100, 1300);
    stopIntake();
    quickTurnTo(-45,100);
    openWings();
    quickMoveDistance(-18, 100, 800);
    quickTurnTo(-90, 100);
    quickMoveDistance(-30, 100, 1000);
    closeWings();
    quickMoveDistance(15, 100, 800);
    quickTurnTo(-240,  100);
    intakeTriball(-100);
    quickMoveDistance(30, 100, 1100);
    quickMoveDistance(-18, 100, 1000);
    quickTurnTo(25, 100);
    intakeTriball(100);
    quickMoveDistance(52, 100, 1400);
    quickTurnTo(90, 100);
    quickMoveDistance(26, 100, 850);
    quickTurnTo(180, 100);
    openWings();
    intakeTriball(-100);
    quickMoveDistance(40, 100, 1100);
    closeWings();
    stopIntake();
    quickMoveDistance(-10, 100, 500);
    quickTurnTo(90, 100); 
    */
    //closeside wp:
    /*openWings();
    wait(500, msec);
    quickTurnTo(-65, 100);
    closeWings();
    wait(200, msec);
    quickTurnTo(-155, 100); 
    intakeTriball(-100); 
    wait(200, msec); 
    quickMoveDistance(40, 100, 1300);
    quickMoveDistance(-10, 100, 900);
    quickTurnTo(10, 100);
    quickMoveDistance(30, 100, 1000);
    quickTurnTo(-45, 100); 
    quickMoveDistance(36, 100, 1300);*/


    //closeside defense auton:
    /*openWings();
    intakeTriball(100);
    quickMoveDistance(50, 100, 1400);
    quickMoveDistance(-16, 100, 1000);
    closeWings();
    quickTurnTo(85, 100);
    quickMoveDistance(10, 100, 600); 
    openWings();
    quickMoveDistance(-35, 100, 1200);
    quickMoveDistance(10, 100, 800);
    closeWings();
    quickTurnTo(-10, 100);
    quickMoveDistance(-45, 100, 1200);
    quickTurnTo(90, 100);
    intakeTriball(-100);
    quickMoveDistance(36, 100, 1400);
    quickMoveDistance(-45, 100, 1200);
*/


}