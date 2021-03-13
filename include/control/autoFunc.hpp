#ifndef _AUTOFUNC_HPP_
#define _AUTOFUNC_HPP_

#include "main.h"
#include "control/visionTracking.hpp"
#include "partsHpp/chassis.hpp"

static double h;
static double gyro_x;
static double gyro_y;

int get_current_time();

double getFt(double dist);

double getDegs(double degs);

void tare();

void brake();

void brake_brake();

//void chaseBall();

void tareChassis();

void collectBall();

void spinLift(int time);

void out_take(int time);

void liftDown(int time);

void spinIntakes(int time);

void driveTime(int time, int pwr);

void filterHeading(void*);

double get_h();

double calcKalman(double U);

void inertialTurn(double target);

void inertialDrive(double target);

void visionDrive(double target);

#endif
