#include "control/autoFunc.hpp"
#include "control/visionTracking.hpp"
#include "partsHpp/chassis.hpp"
#include "partsHpp/liftake.hpp"
#include "main.h"

//PID constants
#define K_P 0.5
#define K_I 0
#define K_D 0
static double error;
static double pwr;
static double lastError = 0;
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
double Q = 2.0; //estimated covariances
double U_hat = 0.0; //estimated state
static double h;
static double h0;

//bool flag1 = true;

//builds main chassis controller
auto chassis = ChassisControllerBuilder()
	.withMotors(10, -1)
	// Green gearset, 3.25 in wheel diam, 10.7 in wheel track
	.withDimensions(AbstractMotor::gearset::green, {{3.25_in, 10.7_in}, imev5GreenTPR})
	.build();

//builds slow chassis controller
auto slowChassis = ChassisControllerBuilder()
	.withMotors(10, -1)
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

//turns using filtered imu data with PID loops
void inertialTurn(double target){
  h0 = h;
  while(true){
	  error = target + h0 - h;
		std::cout << h << std::endl;
    _integral += error;
    _derivative = error - lastError;
    pOut = K_P * error;
    iOut = K_I * _integral;
    dOut = K_D * _derivative;
    pwr = pOut + iOut + dOut;
    chassisManualDrive(pwr, -pwr);
		//std::cout << error << std::endl;
    lastError = error;
    if(abs(error) <= 0.05){
      chassisManualDrive(0, 0);
      break;
    }
    c::delay(5);
  }
}
