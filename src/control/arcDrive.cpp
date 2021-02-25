#include "control/arcDrive.hpp"
#include "control/tracking.hpp"
#include "control/autoFunc.hpp"

#include "arcDrive.hpp"
#include <cmath>
#include <iostream>

#define WHEELBASE_WIDTH 10.94

//center of circle that models arclength
double center[2];

//calculates arclength from three coordinate points
double arclength(double pt1[], double pt2[], double pt3[]){
    cramers_rule(pt1, pt2, pt3);
    double r = distance(center, pt1);
    double d = distance(pt1, pt3);
    double theta = law_of_cos(d, r);
    return r * theta;
}

//calculates length the inner turning wheel should turn
double short_arclength(double pt1[], double pt2[], double pt3[]){
    cramers_rule(pt1, pt2, pt3);
    double r = distance(center, pt1) - (WHEELBASE_WIDTH / 2);
    double d = distance(pt1, pt3);
    double theta = law_of_cos(d, r);
    return (r - (WHEELBASE_WIDTH / 2)) * theta;
}

double long_arclength(double pt1[], double pt2[], double pt3[]){
    cramers_rule(pt1, pt2, pt3);
    double r = distance(center, pt1) + (WHEELBASE_WIDTH / 2);
    double d = distance(pt1, pt3);
    double theta = law_of_cos(d, r);
    return (r + (WHEELBASE_WIDTH / 2)) * theta;
}

/*
 * uses cramer's rule to calculate the center of the circle that includes
 * the three points inputted
 */
void cramers_rule(double pt1[], double pt2[], double pt3[]){
    double x0 = pt1[0];
    double y0 = pt1[1];
    double x1 = pt2[0];
    double y1 = pt2[1];
    double x2 = pt3[0];
    double y2 = pt3[1];

    double a = 2 * (x0 - x1);
    double b = 2 * (y0 - y1);
    double c = pow(x0, 2) + pow(y0, 2) - pow(x1, 2) - pow(y1, 2);
    double d = 2 * (x0 - x2);
    double e = 2 * (y0 - y2);
    double f = pow(x0, 2) + pow(y0, 2) - pow(x2, 2) - pow(y2, 2);

    double A[2][2] = {{a, b}, {d, e}};
    double A1[2][2] = {{c, b}, {f, e}};
    double A2[2][2] = {{a, c}, {d, f}};

    center[0] = det(A1) / det(A);
    center[1] = det(A2) / det(A);
}

//calculates the determinant of a 2x2 matrix
double det(double (&mat)[2][2]){
    return (mat[0][0] * mat[1][1]) - (mat[0][1] * mat[1][0]);
}

//calculates distance between two inputted points
double distance(double pt1[2], double pt2[2]){
    return sqrt(pow(pt2[0] - pt1[0], 2) + pow(pt2[1] - pt1[1], 2));
}

//uses the law of cosines to calculate the angle of the arc used
double law_of_cos(double d, double r){
    double theta = acos(1 - (pow(d, 2) / (2 * pow(r, 2))));
    return theta;
}
