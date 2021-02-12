#include "control/visionTracking.hpp"

#define RED_BALL 1
//needs to be tuned
#define TURN_P 0.7

/*
float finalpwr;

float turnBias(){
  int x_error = vCalc().x_middle_coord - (VISION_FOV_WIDTH / 2);
  if(x_error == 0.1)
    finalpwr = 0;
  else
    finalpwr = x_error * TURN_P;
  return finalpwr;
}

vision_object_s_t vData;
bool flag;

vision_object_s_t vCalc(){
  flag = true;
  return vData;
}

void vMonitor(void*){
  pros::Vision vision(6);
  while(true){
    c::delay(5);
    if(flag){
      vData = vision.get_by_sig(0, RED_BALL);
      flag = false;
    }
  }
}
*/
