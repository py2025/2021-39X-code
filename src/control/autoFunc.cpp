#include "control/autoFunc.hpp"
#include "control/visionTracking.hpp"
#include "partsHpp/chassis.hpp"
#include "partsHpp/liftake.hpp"

//PID gains for InertialTurn
#define K_D 0.04

//PID gains for InertialDrive (driving)
#define K_P1 0.9
#define K_I1 0.0000005
#define K_D1 0.016

//Also PID gains for InertialDrive (turning)
#define K_P2 2
#define K_I2 0.002
#define K_D2 0

//Kalman constants
#define R 40.0 //noise covariance
#define H 1.0 //measurement map scalar
double P = 0.0; //error covariance
double K = 0.0; //kalman gain
double Q = 1.0; //estimated covariances
double U_hat = 0.0; //estimated state

//continued inertialTurn gains
static float K_P = 0.8;
static float K_I = 0.0013;
static float tBias = 0.25;

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

int getTime(){
  return (int) pros::millis();
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
  chassisManualDrive(pwr * 0.98, pwr);
  c::delay(time);
  chassisManualDrive(0, 0);
}

//Gets filtered heading from inertial sensor input and kalman filter
void filterHeading(void*){
	//initialize then reset imu
	pros::Imu inertial(4);
	inertial.reset();
	c::delay(2500);
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

	if(abs(target) < 90 && abs(target) > 45){
		K_P = 1.38;
		K_I = 0.0025;
		tBias = 0.25;
	}
  else if(abs(target) == 45){
    K_P = 1.41;
    K_I = 0.0026;
    tBias = 0.25;
  }
	else if(abs(target) < 45){
		K_P = 1.42;
		K_I = 0.00255;
		tBias = 1.0;
	}
	else if(target == 90){
		K_P = 0.90;
		K_I = 0.00143;
		tBias = 1.0;
	}
  else if(target < 0 && target >= -45){
    K_P = 0.85;
    K_I = 0.00134;
    tBias = 0.50;
  }
  else if(target == -90){
    K_P = 0.88;
		K_I = 0.00139;
		tBias = 1.0;
  }

  while(true){
	  error = target + h0 - h;
		std::cout << error << std::endl;
    _integral += error;
    _derivative = error - lastError;
    pOut = K_P * error;
    iOut = K_I * _integral;
    dOut = K_D * _derivative;
    pwr = pOut + iOut + dOut;
		//slow turn = fun
		if(pwr > 100) pwr = 100; //previously 68
		else if(pwr < -100) pwr = -100; //previously -68
    chassisManualDrive(pwr, -pwr);
    lastError = error;
    if(abs(error) <= tBias){
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
		leftPos = -leftDrive.get_position();
		rightPos = rightDrive.get_position();
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
		if(pwrD > (127 - abs(pwrT))) pwrD = 127 - abs(pwrT);
		else if(pwrD < (-127 + abs(pwrT))) pwrD = -127 + abs(pwrT);
    //if above conditional doesn't work,
    /*
    if(pwrD > 100) pwrD = 100; //used to limit to 90
    else if(pwrD < -100) pwrD -100; //used to limit to -90
    */
		chassisManualDrive(pwrD + pwrT, pwrD - pwrT);
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
