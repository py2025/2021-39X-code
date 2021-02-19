#include "main.h"
#include "control/autoRoutines.hpp"
#include "control/autoFunc.hpp"
#include "partsHpp/chassis.hpp"
#include "partsHpp/liftake.hpp"

//builds main chassis controller
auto chassis = ChassisControllerBuilder()
	.withMotors(-10, 1)
	// Green gearset, 3.25 in wheel diam, 10.7 in wheel track
	.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10.94_in}, imev5GreenTPR})
	.build();

//builds slow chassis controller
auto slowChassis = ChassisControllerBuilder()
	.withMotors(-10, 1)
	// Green gearset, 3.25 in wheel diam, 10.7 in wheel track
	.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10.94_in}, imev5GreenTPR})
	.withMaxVelocity(90)
	.build();

//used in driver control
void startLeft(){
	chassis->moveDistance(1_ft);
	chassis->turnAngle(135_deg);
	driveTime(1000, 100);
	spinLift(1000);
}
//main skills routine
void skills(){
  //all tasks initiate intakes with the hopes of descoring a ball
	Task descore1(intakeT, (void*)1000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	driveTime(750, 127);
	spinLift(1250);
	chassis->moveDistance(-1_ft);
	c::delay(300);
	chassis->turnAngle(95_deg);
	driveTime(500, -127);
	c::delay(300);
	chassis->moveDistance(2.3_ft);
	chassis->turnAngle(91_deg);
	Task intake1(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->moveDistance(3.5_ft);
	chassis->moveDistance(-2.2_ft);
	chassis->turnAngle(49_deg);
	Task descore2(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	driveTime(1500, 127);
	spinLift(1250);
	chassis->moveDistance(-2_ft);
	chassis->turnAngle(125_deg);
	//backs into wall
	driveTime(1250, -127);
	c::delay(1000);
	Task intake2(intakeT, (void*)10000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->moveDistance(11_ft);
	chassis->moveDistance(-1.5_ft);
	chassis->turnAngle(-50_deg);
	Task descore3(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	driveTime(1500, 127);
	spinLift(1250);
}

void skills1(){
	brake();
  inertialDrive(1.25);
  c::delay(500);
  inertialTurn(90);
  inertialTurn(45);
  driveTime(900, 80);
  spinLift(1250);
  inertialDrive(-1.4);
  c::delay(500);
  slowChassis->turnAngle(135_deg);
  inertialDrive(-1.5);
	driveTime(500, -80);
  c::delay(300);
  Task intake0(intakeT, (void*) 3500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  inertialDrive(5.25);
  c::delay(750);
  inertialTurn(90);
  c::delay(1000);
  inertialDrive(0.85);
  spinLift(1250);
  inertialDrive(-1.15);
  c::delay(750);
  inertialTurn(-90);
  inertialDrive(3);
  c::delay(500);
  inertialTurn(90);
  Task intake1(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  Task lift1(liftDelay, (void*)1500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  inertialDrive(1.6);
  c::delay(500);
  inertialDrive(-2);
  c::delay(500);
  inertialTurn(-45);
  inertialDrive(2.8);
  liftDown(350);
  spinLift(1250);
  inertialDrive(-1.4);
  c::delay(500);
  inertialTurn(90);
  inertialTurn(45);
  inertialDrive(-1.4);
	driveTime(300, -80);
  c::delay(500);
  inertialDrive(3.1);
  c::delay(500);
  inertialTurn(90);
  inertialDrive(-1);
  driveTime(750, -80);
  c::delay(750);
  Task intake2(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  inertialDrive(5.2);
  c::delay(500);
  inertialTurn(90);
  inertialDrive(2.7);
  spinLift(1250);
  inertialDrive(-2);
  c::delay(500);
  inertialTurn(-60);
  Task intake3(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  inertialDrive(5.5);
  spinLift(1300);
  Task intake4(intakeT, (void*)1000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  chassis->turnAngle(-180_deg);
  inertialDrive(2);
}

//main match autons
void matchAutonL(){
	chassis->moveDistance(2.5_ft);
	chassis->turnAngle(-123_deg);
	driveTime(1000, 80);
	liftDown(250);
	spinLift(1000);
	slowChassis->moveDistance(-2.5_ft);
	chassis->turnAngle(-45_deg);
	Task intake(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->moveDistance(5_ft);
	chassis->turnAngle(10_deg);
	liftDown(250);
	spinLift(1000);
}

void matchAutonR(){
	chassis->moveDistance(2.5_ft);
	chassis->turnAngle(126_deg);
	driveTime(1000, 80);
	liftDown(250);
	spinLift(1000);
	slowChassis->moveDistance(-2.5_ft);
	chassis->turnAngle(62_deg);
	Task intake(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->moveDistance(5_ft);
	driveTime(500, 80);
	liftDown(250);
	spinLift(1000);
}
