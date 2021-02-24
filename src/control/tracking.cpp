#include "control/tracking.hpp"
#include "control/autoFunc.hpp"
#include "partsHpp/liftake.hpp"
#include "main.h"

#include <cmath>

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
  h_last = start[3];
  while(true){
    avg = (rightPos() + leftPos()) / 2;

    dist = avg / SCONVERSION_IN;

    delta_x = dist * cos(h - h_last);
    delta_y = dist * sin(h - h_last);

    x_pos += delta_x;
    y_pos += delta_y;

    h_last = h;
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
