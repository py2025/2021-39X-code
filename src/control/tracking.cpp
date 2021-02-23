#include "control/tracking.hpp"
#include "control/autoFunc.hpp"
#include "partsHpp/liftake.hpp"

#include <cmath>

double x_pos;
double y_pos;
double h_last;

double avg;

double dist;

double delta_x;
double delta_y;

double rightPos(){
  return rightDrive.get_position();
}

double leftPos(){
  return leftDrive.get_position();
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

 /* moveTo function
  * distance is distance from current coordinate to target coordinate
  * rotate to target then move there
  * @param x, y
  * arctan somewhere maybe?
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

void rotate_to(double x, double y){
  double current_x = get_x();
  double current_y = get_y();
  double target_h = get_heading() - (std::atan((y - current_y) / (x - current_x)) * (180 / M_PI));
  inertialTurn(target_h);
}
