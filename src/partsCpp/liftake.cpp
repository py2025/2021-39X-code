#include "main.h"
#include "partsHpp/liftake.hpp"

//intakes balls
void intake(int power){
  leftIntake.move(power);
  rightIntake.move(-power);
}

//intake task
void intakeT(void* param){
  int time = (int) param;
  intake(127);
  c::delay(time);
  intake(0);
}

//lifts balls
void lift(int power){
  liftMotor1.move(power);
  liftMotor2.move(-power);
}

//lift task
void liftT(void* param){
  int time = (int) param;
  lift(127);
  c::delay(time);
  lift(0);
}
