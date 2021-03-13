#include "control/autoFunc.hpp"
#include "control/visionTracking.hpp"
#include "partsHpp/chassis.hpp"
#include "partsHpp/liftake.hpp"

//PID gains for InertialDrive (driving)
#define K_P1 0.7
#define K_I1 0.0000001
#define K_D1 5

//Also PID gains for InertialDrive (turning)
#define K_P2 3.5
#define K_I2 0.002
#define K_D2 0.01

//Kalman constants
#define R 40.0 //noise covariance
#define H 1.0 //measurement map scalar
double P = 0.0; //error covariance
double K = 0.0; //kalman gain
double Q = 1.0; //estimated covariances
double U_hat = 0.0; //estimated state

//inertialTurn gains
static float K_P = 0.75;
static float K_I = 0.0015;
static float K_D = 0.01;
static float tBias = 0.05;

int start_time;

//used for PID killswitch
int get_current_time(){
  return pros::millis();
}

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
  leftDrive1.tare_position();
	rightDrive.tare_position();
  rightDrive1.tare_position();
}

//brakes motors
void brake(){
	leftDrive.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  leftDrive1.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	rightDrive.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  rightDrive1.set_brake_mode(E_MOTOR_BRAKE_HOLD);
}

void brake_brake(){
	leftDrive.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  leftDrive1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	rightDrive.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  rightDrive1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
}

/*
//uses vision tracking to follow ball
void chaseBall(){
  while(true){
    c::delay(5);
    chassisManualDrive(-turnBias(), turnBias());
    if(abs(turnBias()) <= 5.0){
      break;
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
  spin_intake(127);
  c::delay(time);
  spin_intake(0);
}

//outtake (time based)
void out_take(int time){
  spin_intake(-127);
  c::delay(time);
  spin_intake(0);
}

//spin lift (time based)
void spinLift(int time){
  lift(127);
  c::delay(time);
  lift(0);
}

//lowers lift for a certain amount of time
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

//Gets filtered heading from inertial sensor input and kalman filter
void filterHeading(void*){
	//initialize then tare imu
	pros::Imu inertial(4);
	inertial.reset();
  c::delay(2500);
  while(true){
    	h = calcKalman(inertial.get_rotation());
			c::delay(5);
  }
}

double get_h(){
  return h;
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
  start_time = get_current_time();


	if(target < 90 && target > 45){
    K_P = 0.5;
		K_I = 0.003;
    K_D = 0.025;
		tBias = 0.1;
	}

	if(target > -90 && target < -45){
    K_P = 1.5;
		K_I = 0.0023;
    K_D = 0.5;
		tBias = 0.1;
	}

  else if(target > 0 && target < 45){
    K_P = 0.7;
		K_I = 0.005;
    K_D = 0.025;
		tBias = 0.1;
  }

  else if(target == 45){
    K_P = 1.5;
		K_I = 0.0023;
    K_D = 0.5;
		tBias = 0.1;
  }

	else if(target == -45){
    K_P = 1.5;
		K_I = 0.0023;
    K_D = 0.5;
		tBias = 0.1;
	}

  //done
	else if(target == 90){
    K_P = 1.03;
    K_I = 0.00132;
    K_D = 0.63;
		tBias = 0.10;
	}

  else if(target < 0 && target > -45){
    K_P = 1.1;
		K_I = 0.005;
    K_D = 0.025;
		tBias = 0.1;
  }

  else if(target == -90){
    K_P = 1.033;
    K_I = 0.00136;
    K_D = 0.6;
		tBias = 0.10;
  }

  while(true){
	  error = target + h0 - h;
    _integral += error;
    _derivative = error - lastError;
    pOut = K_P * error;
    iOut = K_I * _integral;
    dOut = K_D * _derivative;
    pwr = pOut + iOut + dOut;
		//slow turn = fun
    /*
		if(pwr > 90) pwr = 90; //previously 68
		else if(pwr < -90) pwr = -90; //previously -68
    */
    chassisManualDrive(-pwr, pwr);
    lastError = error;
    if(abs(error) <= tBias || get_current_time() - start_time >= 1500){
      chassisManualDrive(0, 0);
			brake();
      break;
    }
    c::delay(5);
  }
}

//drive using IMU and motor encoders
//wiggle wiggle wiggle
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
	double _tDerivative = 0;
	double _tIntegral = 0;
	double lastErrorT = 0;

	tare();
	while(true){
		leftPos = (-leftDrive.get_position() + leftDrive1.get_position()) / 2;
		rightPos = (-rightDrive.get_position() + rightDrive1.get_position()) / 2;
		avg = (leftPos + rightPos) / 2;
		errorD = getFt(target) - avg;
		_integral += errorD;
		_derivative = errorD - lastError;
		pwrD = (K_P1 * errorD) + (K_I1 * _integral) + (K_D1 * _derivative);

		errorT = h0 - h;
		_tIntegral += errorT;
		_tDerivative = errorT - lastErrorT;
		pwrT = (K_P2 * errorT) + (K_I2 * _tIntegral) + (K_D2 * _tDerivative);

		//drive power is limited to allow turn power to have an effect
    /*
		if(pwrD > (127 - abs(pwrT))) pwrD = 127 - abs(pwrT);
		else if(pwrD < (-127 + abs(pwrT))) pwrD = -127 + abs(pwrT);
    */

    //if above conditional doesn't work,
    if(pwrD > 90) pwrD = 90; //used to limit to 90
    else if(pwrD < -90) pwrD -90; //used to limit to -90
		chassisManualDrive(pwrD - pwrT, pwrD + pwrT);
		lastError = errorD;
		lastErrorT = errorT;
		if(abs(errorD) <= 10){
			chassisManualDrive(0, 0);
      brake();
			break;
		}
		c::delay(5);
	}
}

/*
void visionDrive(double target){
	double lastError = 0;
	double error = 0;
	double _integral = 0;
	double _derivative = 0;
	double leftPos = 0;
	double rightPos = 0;
	double avg = 0;
	double pwr = 0;

	tare();
	while(true){
		leftPos = abs(leftDrive.get_position());
		rightPos = abs(rightDrive.get_position());
		avg = (leftPos + rightPos) / 2;
		error = getFt(target) - avg;
		_integral += error;
		_derivative = error - lastError;
		pwr = (K_P1 * error) + (K_I1 * _integral) + (K_D1 * _derivative);
		if(pwr > 90) pwr = 90;
		else if(pwr < -90) pwr = -90;
		chassisManualDrive(pwr + turnBias(), pwr - turnBias());
		lastError = error;
		if(abs(error) <= 10){
			chassisManualDrive(0, 0);
			brake();
			break;
		}
		c::delay(5);
	}
}
*/
