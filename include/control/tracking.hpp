#ifndef _TRACKING_HPP_
#define _TRACKING_HPP_
#include "api.h"
#include "main.h"

void init_pos(bool flag);

void test_init(double x, double y);

double leftPos();

double rightPos();

void odom(void*);

double get_x();

double get_y();

double left_filter(double U);

double right_filter(double U);

double get_heading(double x, double y, bool intake);

void move_to(double x, double y, bool intake);

void rotate_to(double x, double y);

/*
double encoder_filter(double U);

double encoder_filter1(double U);
*/

void curve_path(double pt1[2], double pt2[2], bool intake);
#endif
