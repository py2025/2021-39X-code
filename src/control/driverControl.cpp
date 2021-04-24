#include "control/driverControl.hpp"
#include "control/autoFunc.hpp"
#include "control/autoRoutines.hpp"

//Motors + Ports
pros::Motor leftDrive(10);
pros::Motor leftDrive1(7);
pros::Motor rightDrive(6);
pros::Motor rightDrive1(5);
pros::Motor liftMotor1(9);
pros::Motor liftMotor2(3);
pros::Motor leftIntake(2);
pros::Motor rightIntake(8);

pros::Controller mainController(E_CONTROLLER_MASTER);
int rightY;
int leftY;
int BackRU;
int BackRD;
int BackLU;
int BackLD;
int drive_x;
int drive_y;
int a;

//main drive task
void manualChassis(void*){
  while(true){
    c::delay(5);
		leftY = (int) mainController.get_analog(ANALOG_LEFT_Y);
    rightY = (int) mainController.get_analog(ANALOG_RIGHT_Y);
    drive_x = mainController.get_digital(DIGITAL_X);
    drive_y = mainController.get_digital(DIGITAL_Y);
    a = mainController.get_digital(DIGITAL_A);
    chassisManualDrive(rightY, leftY);
    //center descore
    if(drive_x){
      chassisManualDrive(127, 127);
      c::delay(500);
      chassisManualDrive(-100, -127);
      c::delay(500);
      chassisManualDrive(0, 0);
    }
    if(a){
      chassisManualDrive(127, 127);
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
    if(BackRU) spin_intake(-127);
    else if(BackRD) spin_intake(127);
    else spin_intake(0);
    if(BackLU) lift(-127);
    else if(BackLD) lift(127);
  	else lift(0);
  }
}
