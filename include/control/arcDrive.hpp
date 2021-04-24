#ifndef _ARCDRIVE_HPP_
#define _ARCDRIVE_HPP_

#include "main.h" 

double *get_arr(double x, double y);

double arclength(double pt1[], double pt2[], double pt3[]);

double short_arclength(double pt1[], double pt2[], double pt3[]);

double long_arclength(double pt[], double pt2[], double pt3[]);

void cramers_rule(double pt1[], double pt2[], double pt3[]);

double det(double (&mat)[2][2]);

double distance(double pt1[2], double pt2[2]);

double law_of_cos(double d, double r);

double det(double (&mat)[2][2]);

#endif
