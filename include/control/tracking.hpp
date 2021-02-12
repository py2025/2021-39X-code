#ifndef _TRACKING_HPP_
#define _TRACKING_HPP_
#include "api.h"
#include "main.h"

double avg;
double dif;

double dist;

double dx;
double dy;
double dtheta;

double encX;
double encY;
double encHeading;

void positionTracking(double x, double y, double heading);
#endif
