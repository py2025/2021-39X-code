#include "control/arcDrive.hpp"
#include "control/tracking.hpp"
#include "control/autoFunc.hpp"

#include "arcDrive.hpp"
#include <cmath>
#include <iostream>

float center[2];

float arclength(float pt1[], float pt2[], float pt3[]){
    cramers_rule(pt1, pt2, pt3);
    float r = distance(center, pt1);
    float d = distance(pt1, pt3);
    float theta = law_of_cos(d, r);
    return r * theta;
}

void cramers_rule(float pt1[], float pt2[], float pt3[]){
    float x0 = pt1[0];
    float y0 = pt1[1];
    float x1 = pt2[0];
    float y1 = pt2[1];
    float x2 = pt3[0];
    float y2 = pt3[1];

    float a = 2 * (x0 - x1);
    float b = 2 * (y0 - y1);
    float c = pow(x0, 2) + pow(y0, 2) - pow(x1, 2) - pow(y1, 2);
    float d = 2 * (x0 - x2);
    float e = 2 * (y0 - y2);
    float f = pow(x0, 2) + pow(y0, 2) - pow(x2, 2) - pow(y2, 2);

    float A[2][2] = {{a, b}, {d, e}};
    float A1[2][2] = {{c, b}, {f, e}};
    float A2[2][2] = {{a, c}, {d, f}};

    center[0] = det(A1) / det(A);
    center[1] = det(A2) / det(A);
}

float det(float (&mat)[2][2]){
    return (mat[0][0] * mat[1][1]) - (mat[0][1] * mat[1][0]);
}

float distance(float pt1[2], float pt2[2]){
    return sqrt(pow(pt2[0] - pt1[0], 2) + pow(pt2[1] - pt1[1], 2));
}

float law_of_cos(float d, float r){
    float theta = acos(1 - (pow(d, 2) / (2 * pow(r, 2))));
    return theta;
}
