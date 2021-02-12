#ifndef _AUTOFUNC_HPP_
#define _AUTOFUNC_HPP_

#include "main.h"
#include "control/visionTracking.hpp"
#include "partsHpp/chassis.hpp"

using namespace okapi;

void skills();

//void chaseBall();

void tareChassis();

void collectBall();

void spinLift(int time);

void spinIntakes(int time);

void driveTime(int time, int pwr);

void filterHeading(void*);

double calcKalman(double U);

void inertialTurn(double target);

#endif
