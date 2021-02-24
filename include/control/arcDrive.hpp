#ifndef _ARCDRIVE_HPP_
#define _ARCDRIVE_HPP_

#include "main.h"

float arclength(float pt1[], float pt2[], float pt3[]);

void cramers_rule(float pt1[], float pt2[], float pt3[]);

float det(float (&mat)[2][2]);

float distance(float pt1[2], float pt2[2]);

float law_of_cos(float d, float r);

float det(float (&mat)[2][2]);

#endif
