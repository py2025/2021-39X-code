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

//x filter constants
#define R_E 30.0 //noise covariance
#define H_E 1.0 //measurement map scalar
double P_E = 0.0; //error covariance
double K_E = 0.0; //kalman gain
double Q_E = 1.0; //estimated covariances
double U_hat_e = 0.0; //estimated state

//y filter constants
#define R_E1 30.0 //noise covariance
#define H_E1 1.0 //measurement map scalar
double P_E1 = 0.0; //error covariance
double K_E1 = 0.0; //kalman gain
double Q_E1 = 1.0; //estimated covariances
double U_hat_e1 = 0.0; //estimated state
double x_pos;
double y_pos;
double h_last;

double avg;

double dist;

double delta_x;
double delta_y;

//used to initialize starting odometry position
double start[2];

//current coordinate array
double current_pos[2];

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
//center of robot
void init_pos(bool flag){
 if(flag){
   start[0] = 24; //x (in)
   start[1] = 9.5; //y (in) change to front of bot
 }
 else{
   start[0] = 120; //x (in)
   start[1] = 9.5; //y (in)
 }
}

void test_init(double x, double y){
  start[0] = x;
  start[1] = y;
}

void tracking_debug(){
  //std::cout << get_h() << std::endl;
  //std::cout << avg << std::endl; //+ rightDrive1.get_position() / 2 << std::endl;
  std::cout << "x pos: " << get_x() << " y pos: " << get_y() << std::endl;
}

//averages of IMEs
double leftPos(){
  return left_filter(-leftDrive1.get_position());
}

double rightPos(){
  return right_filter(rightDrive.get_position());
}

//tracks distance moved in inches and heading in degrees
void odom(void*){
  tare();
  x_pos = start[0];
  y_pos = start[1];
  h_last = get_h();
  double d_dist = 0;
  double dist_last = 0;
  while(true){
    avg = (leftPos() + rightPos()) / 2;

    dist = avg / SCONVERSION_IN;

    d_dist = dist - dist_last;

    delta_x = d_dist * sin(get_h() - h_last);
    delta_y = d_dist * cos(get_h() - h_last);

    x_pos += delta_x;
    y_pos += delta_y;

    h_last = get_h();
    dist_last = dist;

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

//encoder-Kalman filter calculations
double left_filter(double U){
  K_E = P_E * H_E / (H_E * P_E * H_E + R_E);
  U_hat_e = U_hat_e + K_E * (U - H_E * U_hat_e);
  P_E = (1 - K_E * H_E) * P_E + Q_E;
  return U_hat_e;
}

double right_filter(double U){
  K_E1 = P_E1 * H_E1 / (H_E1 * P_E1 * H_E1 + R_E1);
  U_hat_e1 = U_hat_e1 + K_E1 * (U - H_E1 * U_hat_e1);
  P_E1 = (1 - K_E1 * H_E1) * P_E1 + Q_E1;
  return U_hat_e1;
}

 /* move_to function
  * distance is distance from current coordinate to target coordinate
  * rotate to target then move there
  * @param x, y, intake
 */

void move_to(double x, double y, bool intake){
  double current_x = get_x();
  double current_y = get_y();
  double dist = sqrt(pow(x - current_x, 2) + pow(y - current_y, 2));

  rotate_to(x, y);
  c::delay(200);
  if(intake) spin_intake(127);
  inertialDrive(dist);
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

/*input two points, robot will strafe-drive with the first point as a stop and
 *the second point as the end
 */
 //with odom
/*
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

  double bias = 0;
  //rotate to tangent heading to curve then delay 200;
  while(true){
    leftAvg = (leftDrive.get_position() + leftDrive1.get_position() / 2);
    rightAvg = (rightDrive.get_position() + rightDrive1.get_position()) / 2;

    errorL = ((pt1[1] > current_pos[1]) ? long_alength : short_alength) - (leftAvg / SCONVERSION_IN); //long_alength - (leftAvg / SCONVERSION_IN);
    _integralL += errorL;
    _derivativeL = errorL - lastErrorL;
    pwrL = (KP_L * errorL) + (KI_L * _integralL) + (KD_L * _derivativeL);
    errorR = ((pt1[1] > current_pos[1]) ? short_alength : long_alength) - (rightAvg / SCONVERSION_IN); //short_alength - (rightAvg / SCONVERSION_IN);
    _integralR += errorR;
    _derivativeR = errorR - lastErrorR;
    pwrR = (KP_R * errorR) + (KI_R * _integralR) + (KD_R * _derivativeR);

    (pt1[1] > current_pos[1]) ? ((pwrL > 127) ? (bias = 127 / pwrL) : bias = 1) : ((pwrR > 127) ? (bias = 127 / pwrR) : bias = 1);

    chassisManualDrive(pwrR * bias, pwrL * bias);
  	lastErrorR = errorR;
  	lastErrorL = errorL;
  	if(abs((pt1[1] > current_pos[1]) ? errorL : errorR) <= 0.05){
  		chassisManualDrive(0, 0);
  		brake();
  		break;
  	}
  	c::delay(10);
  }
}
*/

//without odom
void curve_path(double pt1[2], double pt2[2], bool intake){
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
  double bias = 0;
  if(intake) spin_intake(127);
  tare();
  //rotate to tangent heading to curve then delay 200;
  while(true){
    leftAvg = (-leftDrive.get_position() + -leftDrive1.get_position() / 2);
    rightAvg = (rightDrive.get_position() + rightDrive1.get_position()) / 2;

    errorL = ((pt1[1] > 0) ? long_alength : short_alength) - (leftAvg / SCONVERSION_IN); //long_alength - (leftAvg / SCONVERSION_IN);
    _integralL += errorL;
    _derivativeL = errorL - lastErrorL;
    pwrL = (KP_L * errorL) + (KI_L * _integralL) + (KD_L * _derivativeL);
    errorR = ((pt1[1] > 0) ? short_alength : long_alength) - (rightAvg / SCONVERSION_IN); //short_alength - (rightAvg / SCONVERSION_IN);
    _integralR += errorR;
    _derivativeR = errorR - lastErrorR;
    pwrR = (KP_R * errorR) + (KI_R * _integralR) + (KD_R * _derivativeR);

    (pt1[1] > current_pos[1]) ? ((pwrL > 127) ? (bias = 127 / pwrL) : bias = 1) : ((pwrR > 127) ? (bias = 127 / pwrR) : bias = 1);

    chassisManualDrive(pwrR * bias, pwrL * bias);
  	lastErrorR = errorR;
  	lastErrorL = errorL;
  	if(abs((pt1[1] > 0) ? errorL : errorR) <= 0.5){
  		chassisManualDrive(0, 0);
  		brake();
  		break;
  	}
  	c::delay(10);
  }
  spin_intake(0);
}
