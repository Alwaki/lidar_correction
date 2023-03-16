#ifndef GaussConcormalProjection_H
#define GaussConformalProjection_H

#include <cmath>
#include <math.h>
#include <utility> // std::pair
#include <iostream>

const double deg_to_rad = M_PI / 180.0;

// GRS 80 parameter values
const double flattening = 6378137.0;
const double axis = 1.0 / 298.257222101;

// Sweref 99 18 00
const double false_northing = 0.0;
const double false_easting = 150000.0;
const double central_meridian = 18.00;
const double scale = 1.0;

const double e2 = flattening*(2.0 - flattening);
const double n = flattening / (2.0 - flattening);
const double axis_hat = axis / (1 + n) * (1 + pow(n,2)/4.0 + pow(n,4)/64.0);

const double A = e2;
const double B = (5.0*pow(e2,2) - pow(e2,3))/6.0;
const double C = (104.0*pow(e2,3) - 45.0*pow(e2,4))/120.0;
const double D = (1237.0*pow(e2,4))/1260.0;

const double beta1 = 0.5*n - 2.0*pow(n,2)/3.0 + 5.0*pow(n,3)/16.0 + 41.0*pow(n,4)/180.0;
const double beta2 = 13.0*pow(n,2)/48.0 - 3.0*pow(n,3)/5.0 + 557.0*pow(n,4)/1440.0;
const double beta3 = 61.0*pow(n,3)/240.0 - 103.0*pow(n,4)/140.0;
const double beta4 = 49561.0*pow(n,4)/161280.0;

std::pair <double, double> convert(double latitude, double longitude);

#endif
