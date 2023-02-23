#include "GaussConformalProjection.h"


std::pair <double, double> convert(double latitude, double longitude)
{
    

    double phi = latitude * deg_to_rad;
    double lambda = longitude * deg_to_rad;
    double lambda_zero = central_meridian * deg_to_rad;
    
    double phi_star = phi - sin(phi)*cos(phi)*(A + 
                        B*pow(sin(phi),2) + 
                        C*pow(sin(phi),4) + 
                        D*pow(sin(phi),6));

    double delta_lambda = lambda - lambda_zero;
    double xi = atan(tan(phi_star) / cos(delta_lambda));
    double eta = atanh(cos(phi_star) * sin(delta_lambda));
    double x = scale * axis_hat * (xi +
                    beta1 * sin(2.0 * xi) * cosh(2.0 * eta) +
                    beta2 * sin(4.0 * xi) * cosh(4.0 * eta) +
                    beta3 * sin(6.0 * xi) * cosh(6.0 * eta) +
                    beta4 * sin(8.0 * xi) * cosh(8.0 * eta)) +
                    false_northing;
    double y = scale * axis_hat * (eta +
                    beta1 * cos(2.0 * xi) * sinh(2.0 * eta) +
                    beta2 * cos(4.0 * xi) * sinh(4.0 * eta) +
                    beta3 * cos(6.0 * xi) * sinh(6.0 * eta) +
                    beta4 * cos(8.0 * xi) * sinh(8.0 * eta)) +
                    false_easting;

    x = round(x * 1000) / 1000;
    y = round(y * 1000) / 1000;
    return {x,y};                
}

/*
int main()
{   
    auto a = convert(45, 45);
    auto b = convert(45, 47);
    auto c = convert(67, 89);
    std::cout << a.first << " " << a.second << "\n";
    std::cout << b.first << " " << b.second << "\n";
    std::cout << c.first << " " << c.second << "\n";
    return 0;
}
*/