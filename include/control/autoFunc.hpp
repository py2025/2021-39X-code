#ifndef _AUTOFUNC_HPP_
#define _AUTOFUNC_HPP_

#include "main.h"
#include "control/visionTracking.hpp"
#include "partsHpp/chassis.hpp"

using namespace okapi;

double getFt(double dist);

double getDegs(double degs);

void tare();

void brake();

void startLeft();

void skills();

void skills1();

//void chaseBall();

void tareChassis();

void collectBall();

void spinLift(int time);

void liftDown(int time);

void spinIntakes(int time);

void driveTime(int time, int pwr);

void filterHeading(void*);

double calcKalman(double U);

void inertialTurn(double target);

void inertialDrive(double target);

void matchAutonL();

void matchAutonR();

#endif
