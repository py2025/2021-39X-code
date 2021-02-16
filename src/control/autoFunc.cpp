#include "control/autoFunc.hpp"
#include "control/visionTracking.hpp"
#include "partsHpp/chassis.hpp"
#include "partsHpp/liftake.hpp"
#include "main.h"

//PID constants
#define K_P 0.9 //1.35
#define K_I 0.00185 //0.0015 //0.001
#define K_D 0.43 //2.0 //0.9

#define K_P1 0.9
#define K_I1 0
#define K_D1 0

#define K_P2 0.9

//Kalman constants
#define R 30.0 //noise covariance
#define H 1.0 //measurement map scalar
double P = 0.0; //error covariance
double K = 0.0; //kalman gain
double Q = 1.0; //estimated covariances
double U_hat = 0.0; //estimated state
static double h;

//bool flag1 = true;

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

//tares motors
void tare(){
	leftDrive.tare_position();
	rightDrive.tare_position();
}

//brakes motors
void brake(){
	leftDrive.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	rightDrive.set_brake_mode(E_MOTOR_BRAKE_HOLD);
}

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
	chassis->moveDistance(1.5_ft);
	chassis->turnAngle(-130_deg);
	driveTime(750, 127);
	spinLift(1250);
	chassis->moveDistance(-1.2_ft);
	chassis->turnAngle(145_deg); //151
	driveTime(1000, -127);
	c::delay(200);
	driveTime(100, 127);
	chassis->turnAngle(5_deg);
	Task intake0(intakeT, (void*)3500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->moveDistance(5.2_ft);
	Task lift0(liftT, (void*) 350, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->turnAngle(-83_deg);
	driveTime(600,85);
	liftDown(350);
	spinLift(1250);
	slowChassis->moveDistance(-1_ft);
	chassis->turnAngle(90_deg);
	chassis->moveDistance(3_ft);
	chassis->turnAngle(-82_deg);
	Task intake1(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	Task lift1(liftDelay, (void*)1500, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	driveTime(750, 127);
	chassis->moveDistance(-2.5_ft);
	chassis->turnAngle(51_deg);
	driveTime(1500, 127);
	liftDown(200);
	spinLift(1250);
	chassis->moveDistance(-1.4_ft);
	chassis->turnAngle(-127.81_deg);
	driveTime(1250, -127);
	chassis->moveDistance(3.2_ft);
	chassis->turnAngle(-82_deg);
	Task intake2(intakeT, (void*)2000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->moveDistance(4_ft);
	chassis->turnAngle(-100_deg);
	chassis->moveDistance(3_ft);
	spinLift(1250);
	chassis->moveDistance(-0.75_ft);
	chassis->turnAngle(90_deg);
	Task intake3(intakeT, (void*)3000, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT);
	chassis->moveDistance(4.3_ft);
	chassis->turnAngle(-45_deg);
	driveTime(750, 127);
	spinLift(1250);
	chassis->turnAngle(-90_deg);
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
  chassisManualDrive(pwr * 0.98, pwr);
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
	double lastError = 0;
	double error = 0;
	double _integral = 0;
	double _derivative = 0;
  double h0 = h;
	double pOut = 0;
	double iOut = 0;
	double dOut = 0;
	double pwr = 0;
  while(true){
	  error = target + h0 - h;
		std::cout << error << std::endl;
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
			brake();
      break;
    }
    c::delay(5);
  }
}

//drive using IMU and motor encoders
void inertialDrive(double target){
	double lastError = 0;
	double errorD = 0;
	double _integral = 0;
	double _derivative = 0;
	double leftPos = 0;
	double rightPos = 0;
	double avg = 0;
	double pwrD = 0;

	double h0 = h;
	double pwrT = 0;
	double errorT = 0;

	double errors[2];
	tare();
	while(true){
		leftPos = abs(leftDrive.get_position());
		rightPos = abs(rightDrive.get_position());
		avg = -(leftPos + rightPos) / 2;
		errorD = getFt(target) - avg;
		_integral += errorD;
		_derivative = errorD - lastError;
		pwrD = (K_P1 * errorD) + (K_I1 * _integral) + (K_D1 * _derivative);

		errorT = h0 - h;
		pwrT = (K_P2 * errorT);
		chassisManualDrive(pwrD + pwrT, pwrD - pwrT);
		lastError = errorD;
		errors[1] = errorD;
		errors[2] = errorT;
		std::cout << errors[1] << " + " << errors[2] << std::endl;
		if(abs(errorD) <= 0.05){
			chassisManualDrive(0, 0);
			brake();
			break;
		}
		c::delay(5);
	}
}

//main match auton
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
