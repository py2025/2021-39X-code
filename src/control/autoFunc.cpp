#include "control/autoFunc.hpp"
#include "control/visionTracking.hpp"
#include "partsHpp/chassis.hpp"
#include "partsHpp/liftake.hpp"
#include "main.h"

//PID constants
#define K_P 0.7 //1.35
#define K_I 0.002 //0.0015 //0.001
#define K_D 0 //2.0 //0.9

double lastError;
double error;
static double pwr;
static double pOut;
static double iOut;
static double dOut;
static double _integral;
static double _derivative;

//Kalman constants
#define R 30.0 //noise covariance
#define H 1.0 //measurement map scalar
double P = 0.0; //error covariance
double K = 0.0; //kalman gain
double Q = 1.0; //estimated covariances
double U_hat = 0.0; //estimated state
static double h;
static double h0;

//bool flag1 = true;

//builds main chassis controller
auto chassis = ChassisControllerBuilder()
	.withMotors(-10, 1)
	// Green gearset, 3.25 in wheel diam, 10.7 in wheel track
	.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10.7_in}, imev5GreenTPR})
	.build();

//builds slow chassis controller
auto slowChassis = ChassisControllerBuilder()
	.withMotors(-10, 1)
	// Green gearset, 3.25 in wheel diam, 10.7 in wheel track
	.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10.7_in}, imev5GreenTPR})
	.withMaxVelocity(90)
	.build();

//converts desired distance to encoder units
double getFt(double dist){
  dist *= SCONVERSION;
  return dist;
}

//converts desired turn to encoder units
double getDegs(double degs){
  degs *= TCONVERSION;
  return degs;
}

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
	chassis->moveDistance(1_ft);
	chassis->turnAngle(-135_deg);
	driveTime(750, 127);
	spinLift(1250);
	chassis->moveDistance(-1.4_ft);
	chassis->turnAngle(148_deg);
	driveTime(1000, -127);
	Task intake0(intakeT, (void*)3500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->moveDistance(5.3_ft);
	Task lift0(liftT, (void*) 350, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->turnAngle(-85_deg);
	driveTime(600, 75);
	liftDown(350);
	spinLift(1250);
	slowChassis->moveDistance(-1_ft);
	chassis->turnAngle(88_deg);
	chassis->moveDistance(3_ft);
	chassis->turnAngle(-80_deg);
	Task intake1(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	Task lift1(liftDelay, (void*)1500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	driveTime(750, 127);
	chassis->moveDistance(-2.5_ft);
	chassis->turnAngle(50_deg);
	driveTime(1500, 127);
	liftDown(200);
	spinLift(1250);
	chassis->moveDistance(-1.4_ft);
	chassis->turnAngle(-125_deg);
	driveTime(1000, -127);
	chassis->moveDistance(3_ft);
	chassis->turnAngle(-84_deg);
	Task intake2(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->moveDistance(4.3_ft);
	chassis->turnAngle(-87_deg);
	chassis->moveDistance(3_ft);
	spinLift(1250);
	chassis->moveDistance(-0.75_ft);
	chassis->turnAngle(87_deg);
	Task intake3(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->moveDistance(4.2_ft);
	chassis->turnAngle(-44_deg);
	driveTime(750, 127);
	spinLift(1250);
	chassis->turnAngle(-88_deg);
}

/*
//uses vision tracking to follow ball
void chaseBall(){
  while(flag1){
    c::delay(5);
    chassisManualDrive(-turnBias(), turnBias());
    if(abs(turnBias()) <= 5.0){
      flag1 = false;
      std::cout << "a" << std::endl;
    }
  }
}
*/

//tare chassis motor encoder units
void tareChassis(){
  leftDrive.tare_position();
  rightDrive.tare_position();
}

//spin intakes (time based)
void spinIntakes(int time){
  intake(127);
  c::delay(time);
  intake(0);
}

//spin lift (time based)
void spinLift(int time){
  lift(127);
  c::delay(time);
  lift(0);
}

void liftDown(int time){
	lift(-127);
	c::delay(time);
	lift(0);
}

//drive (time based)
void driveTime(int time, int pwr){
  chassisManualDrive(pwr, pwr);
  c::delay(time);
  chassisManualDrive(0, 0);
}

//Gets filtered heading from inertial sensor input and kalman filter I guess
void filterHeading(void*){
	//initialize then reset imu
	pros::Imu inertial(4);
	inertial.reset();
	c::delay(3000);
  while(true){
    	h = calcKalman(inertial.get_rotation());
			c::delay(5);
  }
}

//Kalman calculations
double calcKalman(double U){
  K = P * H / (H * P * H + R);
  U_hat = U_hat + K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}

//turns using filtered imu data with PID loops relative to initial turn
//parameter in degs, [+] right turn [-] left turn
void inertialTurn(double target){
	lastError = 0;
	error = 0;
	pwr = 0;
	pOut = 0;
	iOut = 0;
	dOut = 0;
	_integral = 0;
	_derivative = 0;
  h0 = h;
  while(true){
	  error = target + h0 - h;
		std::cout << error << std::endl;
    _integral += error;
    _derivative = error - lastError;
    pOut = K_P * error;
    iOut = K_I * _integral;
    dOut = K_D * _derivative;
    pwr = pOut + iOut + dOut;
    chassisManualDrive(-pwr, pwr);
		//std::cout << error << std::endl;
    lastError = error;
    if(abs(error) <= 0.05){
      chassisManualDrive(0, 0);
      break;
    }
    c::delay(5);
  }
}
