#include "control/autoRoutines.hpp"
#include "control/autoFunc.hpp"
#include "partsHpp/chassis.hpp"
#include "partsHpp/liftake.hpp"

using namespace okapi;

//builds main chassis controller
auto chassis = ChassisControllerBuilder()
	.withMotors(-10, 1)
	// Green gearset, 3.25 in wheel diam, 10.7 in wheel track
	.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10.94_in}, imev5GreenTPR})
	.build();
	//[-] right turn, [+] left turn

//builds slow chassis controller
auto slowChassis = ChassisControllerBuilder()
	.withMotors(-10, 1)
	// Green gearset, 3.25 in wheel diam, 10.7 in wheel track
	.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10.94_in}, imev5GreenTPR})
	.withMaxVelocity(90)
	.build();
	//[-] right turn, [+] left turn

//auto assisted start
void startLeft(){
	chassis->moveDistance(1_ft);
	chassis->turnAngle(135_deg);
	driveTime(1000, 100);
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
  c::delay(300);
  inertialTurn(90);
  inertialTurn(45);
  driveTime(900, 80);
  spinLift(1250);
  inertialDrive(-1.4);
  c::delay(300);
  slowChassis->turnAngle(135_deg);
  inertialDrive(-1.5);
	driveTime(500, -80);
  c::delay(300);
  Task intake0(intakeT, (void*) 3500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  inertialDrive(5.325);
  c::delay(300);
  inertialTurn(90);
  c::delay(300);
  inertialDrive(0.85);
  spinLift(1250);
  inertialDrive(-1.15);
  c::delay(300);
  inertialTurn(-90);
  inertialDrive(3);
  c::delay(300);
  inertialTurn(90);
  Task intake1(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  Task lift1(liftDelay, (void*)1500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  inertialDrive(1.6);
  c::delay(300);
  inertialDrive(-2);
  c::delay(300);
  inertialTurn(-45);
  inertialDrive(2.5);
	driveTime(500, 80);
  liftDown(350);
  spinLift(1250);
  inertialDrive(-1.4);
  c::delay(300);
  inertialTurn(90);
  inertialTurn(45);
  inertialDrive(-1.4);
	driveTime(300, -80);
  c::delay(300);
  inertialDrive(3.1);
  c::delay(300);
  inertialTurn(90);
  inertialDrive(-1);
  driveTime(750, -80);
  c::delay(300);
  Task intake2(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  inertialDrive(5.3);
  c::delay(300);
  inertialTurn(90);
  inertialDrive(2.7);
  spinLift(1250);
  inertialDrive(-2.3);
  c::delay(300);
  inertialTurn(-60);
  Task intake3(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  inertialDrive(5.1);
	driveTime(400, 80);
  spinLift(1300);
  Task intake4(intakeT, (void*)1000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	inertialDrive(-3.86);
	c::delay(300);
	inertialTurn(-35);
	Task intake5(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	inertialDrive(3.3);
	c::delay(200);
	inertialDrive(-1);
	c::delay(300);
	inertialTurn(-38);
	inertialDrive(4.45);
	c::delay(300);
	inertialTurn(-52);
	inertialDrive(0.75);
	driveTime(300, 127);
	spinLift(1300);

	/*
  chassis->turnAngle(-180_deg);
  inertialDrive(2);
	*/
}

//main match autons
void matchAutonL(){
	inertialDrive(2.5);
	c::delay(300);
	inertialTurn(90);
	inertialTurn(42);
	inertialDrive(0.5);
	driveTime(350, 80);
	liftDown(250);
	spinLift(1000);
	inertialDrive(-2.8);
	c::delay(300);
	inertialTurn(57);
	Task intake(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	inertialDrive(5);
	driveTime(500, 80);
	liftDown(250);
	spinLift(1000);
	chassis->moveDistance(-1.5_ft);
}

void matchAutonR(){
	inertialDrive(2.5);
	c::delay(300);
	inertialTurn(-90);
	inertialTurn(-42);
	inertialDrive(0.5);
	driveTime(350, 80);
	liftDown(250);
	spinLift(1000);
	inertialDrive(-2.8);
	c::delay(300);
	inertialTurn(-57);
	Task intake(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	inertialDrive(5);
	driveTime(500, 80);
	liftDown(250);
	spinLift(1000);
	chassis->moveDistance(-1.5_ft);
}

void oneBallR(){
	brake();
  inertialDrive(1.25);
  c::delay(300);
  inertialTurn(90);
  inertialTurn(45);
	spin_intake(127);
  driveTime(500, 70);
	driveTime(300, 50);
	spin_intake(0);
  spinLift(1250);
  inertialDrive(-1.4);
	inertialTurn(-90);
	inertialTurn(-45);
}
