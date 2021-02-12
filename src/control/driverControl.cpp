#include "control/driverControl.hpp"

//Motors + Ports
pros::Motor leftDrive(10);
pros::Motor rightDrive(1);
pros::Motor liftMotor1(9);
pros::Motor liftMotor2(3);
pros::Motor leftIntake(2);
pros::Motor rightIntake(8);

pros::Controller mainController(E_CONTROLLER_MASTER);
int RightY;
int LeftY;
int BackRU;
int BackRD;
int BackLU;
int BackLD;
int x;
int y;

//main drive task
void manualChassis(void*){
  while(true){
    c::delay(5);
		RightY = (int) mainController.get_analog(ANALOG_RIGHT_Y);
    LeftY = (int) mainController.get_analog(ANALOG_LEFT_Y);
    x = mainController.get_digital(DIGITAL_X);
    y = mainController.get_digital(DIGITAL_Y);
    chassisManualDrive(RightY, LeftY);
    if(x){
      chassisManualDrive(127, 127);
      c::delay(500);
      chassisManualDrive(-127, 127);
      c::delay(500);
      chassisManualDrive(0, 0);
    }
  }
}

//main lift task
void lifting(void*){
  while(true){
    c::delay(5);
	  BackRU = mainController.get_digital(DIGITAL_R1);
	  BackRD = mainController.get_digital(DIGITAL_R2);
		BackLU = mainController.get_digital(DIGITAL_L1);
	  BackLD = mainController.get_digital(DIGITAL_L2);
    if(BackRU) intake(-127);
    else if(BackRD) intake(127);
    else intake(0);
    if(BackLU) lift(-127);
    else if(BackLD) lift(127);
  	else lift(0);
  }
}
