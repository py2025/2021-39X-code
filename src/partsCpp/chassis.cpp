#include "main.h"
#include "partsHpp/chassis.hpp"

//Main Drive Method
void chassisManualDrive(int rightPower, int leftPower){
  rightDrive.move(-rightPower);
  rightDrive1.move(rightPower);
  leftDrive.move(-leftPower);
  leftDrive1.move(leftPower);
}
