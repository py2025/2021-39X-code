#include "main.h"
#include "partsHpp/chassis.hpp"

//Main Drive Method
void chassisManualDrive(int leftPower, int rightPower){
  leftDrive.move(-leftPower);
  rightDrive.move(rightPower);
}
