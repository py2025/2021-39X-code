#include "main.h"
#include "partsHpp/chassis.hpp"

//Main Drive Method
void chassisManualDrive(int rightPower, int leftPower){
  leftDrive.move(-leftPower);
  rightDrive.move(rightPower);
}
