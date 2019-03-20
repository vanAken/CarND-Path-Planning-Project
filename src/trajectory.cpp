#include <iostream>
#include <cmath>
#include "trajectory.h"
#include "spline.h"

Trajectory::Trajectory( vector<double> car_x, vector<double> car_y, double car_s, double car_d,
                        double dt, double v_max, double a_max, Frenet frenet)
{   long max_waypoints = 550;
    double dl  = distance(car_x[0],car_y[0],car_x[1],car_y[1]);
    double car_v = dl/dt; // initial velocity

    next_x = {car_x[0], car_x[1]};
    next_y = {car_y[0], car_y[1]};

    // create spline point
    spline_points_x = next_x;
    spline_points_y = next_y;
    spline_points_v = {car_v, car_v};
    vector<double> next_xy;

        for(int i = 0; i < ::next_s.size(); i++) {
            if (car_s+4 < ::next_s[i] ){
                next_xy = frenet.sd_to_xy(::next_s[i], ::next_d[i]);
                spline_points_x.push_back(next_xy[0]);
                spline_points_y.push_back(next_xy[1]);
                spline_points_v.push_back(::next_v[i]);
            }
        }
 
    // frenet s is used as the spline x value
    double s_spline = 0.0;
    vector<double> spline_points_s = {0};
    for(int i = 1; i < spline_points_x.size(); i++) {
        s_spline += distance(spline_points_x[i-1],spline_points_y[i-1],
                             spline_points_x[i],spline_points_y[i]);
        spline_points_s.push_back(s_spline);
    }
    tk::spline spline_x;
    tk::spline spline_y;
    tk::spline spline_v;
    spline_x.set_points( spline_points_s, spline_points_x );
    spline_y.set_points( spline_points_s, spline_points_y );
    spline_v.set_points( spline_points_s, spline_points_v );      
    while( next_x.size() < max_waypoints) {
        if (car_v <= spline_v(16+dl) ) car_v = min(v_max, car_v + a_max/2 * dt);
        else                           car_v = max( 4.  , car_v - a_max/2 * dt);
//        std::cout << "spline_v(32+dl): " << spline_v(32+dl) << "car_v: " << car_v << std::endl;
        dl += car_v * dt; 
        next_x.push_back( spline_x(dl) );
        next_y.push_back( spline_y(dl) );
    } 
// std::cout << "############################################# "<< std::endl;
}
vector<double> Trajectory::next_X() {
    return next_x;
}
vector<double> Trajectory::next_Y() {
    return next_y;
}
