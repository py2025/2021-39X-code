#include "control/tracking.hpp"
#include "control/autoFunc.hpp"
#include "control/arcDrive.hpp"
#include "partsHpp/liftake.hpp"

#include <cmath>

#define KP_L 0.9
#define KI_L 0.0000005
#define KD_L 0.016

#define KP_R 0.9
#define KI_R 0.0000005
#define KD_R 0.016

double x_pos;
double y_pos;
double h_last;

double avg;

double dist;

double delta_x;
double delta_y;

//encoder-Kalman filter constants
#define R_E 40.0 //noise covariance
#define H_E 1.0 //measurement map scalar
double P_E = 0.0; //error covariance
double K_E = 0.0; //kalman gain
double Q_E = 1.0; //estimated covariances
double U_hat_e = 0.0; //estimated state

//used to initialize starting odometry position
double start[3];

//current coordinate array
double current_pos[2];

double rightPos(){
  return rightDrive.get_position();
}

double leftPos(){
  return leftDrive.get_position();
}

/**
 *   ______________
 *  |______________|
 *  |______________|
 *  |______________|
 *  |______________|
 *  |______________|
 *  |__.________.__|
 *[left_start] [right_start]
 */

//true = right start
void init_pos(bool flag){
 if(flag){
   start[1] = 24; //x (in)
   start[2] = 9.5; //y (in) change to front of bot
   //start[3] = ; //figure out starting heading (probably 90 deg)
 }
 else{
   start[1] = 120; //x (in)
   start[2] = 9.5; //y (in)
   //start[3] = ; //figure out starting heading (probably 90 deg)
 }
}

//tracks distance moved in inches and heading in degrees
void odom(void*){
  x_pos = start[1];
  y_pos = start[2];
  h_last = h;
  while(true){
    avg = (rightPos() + leftPos()) / 2;

    dist = avg / SCONVERSION_IN;

    delta_x = dist * cos(h - h_last);
    delta_y = dist * sin(h - h_last);

    x_pos += delta_x;
    y_pos += delta_y;

    h_last = h;

    current_pos[0] = get_x();
    current_pos[1] = get_y();
    c::delay(10);
  }
}

double get_x(){
  return x_pos;
}

double get_y(){
  return y_pos;
}

double get_heading(){
  return h;
}

 /* move_to function
  * distance is distance from current coordinate to target coordinate
  * rotate to target then move there
  * @param x, y, intake
 */

void move_to(double x, double y, bool intake){
  double current_x = get_x();
  double current_y = get_y();
  double distance = sqrt(pow(x - current_x, 2) + pow(y - current_y, 2));

  rotate_to(x, y);
  c::delay(200);
  if(intake) spin_intake(127);
  inertialDrive(distance);
  spin_intake(0);
}

/* rotate_to function
 * turns robot to face target coordinate
 */
void rotate_to(double x, double y){
  double current_x = get_x();
  double current_y = get_y();
  double target_h = get_heading() - (std::atan((y - current_y) / (x - current_x)) * (180 / M_PI));
  inertialTurn(target_h);
}

//encoder-Kalman filter calculations
double encoder_filter(double U){
  K_E = P_E* H_E / (H_E * P_E * H_E + R_E);
  U_hat_e = U_hat_e + K_E * (U - H_E * U_hat_e);
  P_E = (1 - K_E * H_E) * P_E + Q_E;
  return U_hat_e;
}

/*input two points, robot will strafe-drive with the first point as a stop and
 *the second point as the end
 */
void curve_path(double pt1[2], double pt2[2]){
  double leftAvg;
  double rightAvg;
  double left_dist;
  double right_dist;

  double lastErrorL = 0;
  double errorL = 0;
  double _integralL = 0;
  double _derivativeL = 0;
  double pwrL = 0;

  double lastErrorR = 0;
  double errorR = 0;
  double _integralR = 0;
  double _derivativeR = 0;
  double pwrR = 0;

  double short_alength = short_arclength(current_pos, pt1, pt2);
  double long_alength = long_arclength(current_pos, pt1, pt2);
  while(true){
    leftAvg = -leftDrive.get_position();
    rightAvg = rightDrive.get_position();

    errorL = ((pt1[1] > current_pos[1]) ? long_alength : short_alength) - (leftAvg / SCONVERSION_IN); //long_alength - (leftAvg / SCONVERSION_IN);
    _integralL += errorL;
    _derivativeL = errorL - lastErrorL;
    pwrL = (KP_L * errorL) + (KI_L * _integralL) + (KD_L * _derivativeL);
    errorR = ((pt1[1] > current_pos[1]) ? short_alength : long_alength) - (rightAvg / SCONVERSION_IN); //short_alength - (rightAvg / SCONVERSION_IN);
    _integralR += errorR;
    _derivativeR = errorR - lastErrorR;
    pwrR = (KP_R * errorR) + (KI_R * _integralR) + (KD_R * _derivativeR);

    chassisManualDrive(pwrR, pwrL);
  	lastErrorR = errorR;
  	lastErrorL = errorL;
  	if(abs(errorL) <= 0.05){
  		chassisManualDrive(0, 0);
  		brake();
  		break;
  	}
  	c::delay(5);
  }
}
