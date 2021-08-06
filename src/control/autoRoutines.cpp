#include "control/autoRoutines.hpp"
#include "control/autoFunc.hpp"
#include "partsHpp/chassis.hpp"
#include "partsHpp/liftake.hpp"

using namespace okapi;

//builds main chassis controller
auto chassis = ChassisControllerBuilder()
	.withMotors({6, 5}, {-10, -7})
	// Green gearset, 3.25 in wheel diam, 10.7 in wheel track
	.withDimensions(AbstractMotor::gearset::green, {{5.4166_in, 15_in}, imev5GreenTPR})
	.build();
	//[-] right turn, [+] left turn

//builds slow chassis controller
auto slowChassis = ChassisControllerBuilder()
	.withMotors({6, 5}, {-10, -7})
	// Green gearset, 3.25 in wheel diam, 10.7 in wheel track
	.withDimensions(AbstractMotor::gearset::green, {{5.4166_in, 15_in}, imev5GreenTPR})
	.withMaxVelocity(80)
	.build();
	//[-] right turn, [+] left turn

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
	driveTime(300, 80);
  	spinLift(1250);
  	inertialDrive(-1);
	inertialTurn(-90);
	inertialTurn(-30);
	inertialDrive(-1.1);
	driveTime(600, -55);
  	Task intake0(intakeT, (void*) 2700, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	c::delay(200);
  	inertialDrive(5.12);
	c::delay(200);
  	inertialTurn(90);
  	driveTime(450, 60);
  	spinLift(1500);
  	inertialDrive(-1);
	c::delay(100);
  	inertialTurn(-90);
	c::delay(100);
  	inertialDrive(3);
  	inertialTurn(90);
  	Task intake1(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  	Task lift1(liftDelay, (void*)1200, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  	inertialDrive(1.1);
	driveTime(600, 50);
  	inertialDrive(-1.55);
	c::delay(250);
 	inertialTurn(-45);
  	inertialDrive(1.65);
	driveTime(400, 40);
  	spinLift(1000);
  	inertialDrive(-1.6);
  	inertialTurn(45);
	c::delay(100);
	inertialTurn(90);
  	inertialDrive(1.1);
  	inertialTurn(90);
  	inertialDrive(-0.6);
  	driveTime(750, -75);
  	Task intake2(intakeT, (void*)2500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  	inertialDrive(5.15);
  	inertialTurn(90);
  	inertialDrive(2.6);
	driveTime(400, 30);
  	spinLift(1250);
  	inertialDrive(-2.4);
  	inertialTurn(-58.5);
  	Task intake3(intakeT, (void*)1500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  	inertialDrive(5.3);
	driveTime(400, 60);
  	spinLift(1000);
  	inertialDrive(-1.2);
  	chassis->turnAngle(116_deg);
  	inertialDrive(-0.5);
	driveTime(500, -40);
 	Task intake4(intakeT, (void*) 3500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  	inertialDrive(5.1);
  	inertialTurn(90);
  	driveTime(750, 40);
  	spinLift(1250);
  	inertialDrive(-1);
  	inertialTurn(-90);
  	inertialDrive(2.6);
  	inertialTurn(90);
  	Task intake5(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  	Task lift2(liftDelay, (void*)1200, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
  	inertialDrive(1.1);
	driveTime(600, 50);
  	inertialDrive(-2.1);
  	inertialTurn(-45);
  	inertialDrive(2.5);
	driveTime(500, 50);
  	liftDown(350);
  	spinLift(1250);
  	inertialDrive(-1.6);
	c::delay(300);
  	inertialTurn(90);
	inertialTurn(90);
	inertialTurn(15);
	Task intake10(intakeT, (void*)3500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	inertialDrive(4);
	inertialTurn(-45);
	spin_intake(-127);
	driveTime(300, 90);
	chassis->turnAngle(-15_deg);
	driveTime(300, 30);
  	liftDown(350);
  	spinLift(1250);
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

void matchAuto(){
	spinLift(500);
	inertialDrive(2.55);
	c::delay(300);
	inertialTurn(-90);
	Task intake(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	inertialDrive(1.5);
	driveTime(500, 80);
	spinLift(1000);
	out_take(400);
	inertialDrive(-1.75);
	c::delay(300);
	inertialTurn(45);
	c:delay(300);
	inertialDrive(1.25);
	driveTime(500, 90);
	c::delay(300);
	inertialDrive(-9.25);
	c::delay(300);
	inertialTurn(45);
	c::delay(300);
	driveTime(750, -127);
	inertialDrive(1.5);
}
