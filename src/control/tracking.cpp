#include "control/tracking.hpp"

double rightPos(){
  return rightDrive.get_position();
}

double leftPos(){
  return leftDrive.get_position();
}

//tracks distance moved in inches and heading in degrees
void positionTracking(double x, double y, double heading){
  avg = (rightPos() + leftPos()) / 2;
  dif = (rightPos() - leftPos()) / 2;

  dist = avg / SCONVERSION;
  dtheta = dif / TCONVERSION;

  dx = dist * cos(dtheta);
  dy = dist * sin(dtheta);

  heading += dtheta;
  x += dx;
  y += dy;
  if(heading >= 360) heading -= 360;
  else if(heading <= 360) heading += 360;
}
