#include <iostream>
#include <cmath>
#include "trajectory.h"
#include "spline.h"

Trajectory::Trajectory( vector<double> previous_path_x, vector<double> previous_path_y, double v_max, double a_max,
                        Frenet frenet, vector<double>next_s, vector<double>next_d, vector<double>next_v, double dt)
{   const long max_waypoints = 500;
    next_x = {previous_path_x[0],previous_path_x[1]};   // take the first two points ot the old path
    next_y = {previous_path_y[0],previous_path_y[1]};   // for the next Trajectory
    double dl  = distance( next_x[0],next_y[0],next_x[1],next_y[1]);
    double v_t = dl/dt; // initial velocity  

    // create first two spline points from the beginning of the previous_path
    spline_points_x = next_x;
    spline_points_y = next_y;
    spline_points_v = {v_t, v_t};

    // create the next spline points from A* in min 30m
    vector<double> next_xy; // tmp var
    vector<double> next_sd0;
    next_sd0 = frenet.xy_to_sd(spline_points_x[1],spline_points_y[1]);
    for(int i = 0; i < next_s.size(); i++) { // over  6945.554m -15 starts at 0 again
        if (next_sd0[0] + v_t*1.5 < next_s[i]) {  // v_t*1.5 is aound 30m by full speed
            if (1000 < next_sd0[0] || next_s[i] < 6000  || next_sd0[0]+6945 + v_t*1.5 < next_s[i] ){// close loop fix
                if (next_s[i-1]+1 < next_s[i]) {  // check next_s is rising more than 1m
                    next_xy = frenet.sd_to_xy( next_s[i], next_d[i]);
                    spline_points_x.push_back( next_xy[0] );
                    spline_points_y.push_back( next_xy[1] );
                    spline_points_v.push_back( next_v[i]  );
                    next_xy = frenet.sd_to_xy( next_s[i]+10, next_d[i]); // double point to avoid overswinging
                    spline_points_x.push_back( next_xy[0] );
                    spline_points_y.push_back( next_xy[1] );
                    spline_points_v.push_back( next_v[i]  );
                 }
            }
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
        if (v_t <= spline_v(16+dl) ) v_t = min(v_max, v_t + a_max/2 * dt); // +16 for future v
        else                         v_t = max( 4.  , v_t - a_max/2 * dt);
        //std::cout << "spline_v(32+dl): " << spline_v(32+dl) << "v_t: " << v_t << std::endl;
        dl += v_t * dt;
        double next_x_lin = 2*next_x[next_x.size()-1]-next_x[next_x.size()-2]; // next_x_lin = 2*x_2-x_1
        double next_y_lin = 2*next_y[next_y.size()-1]-next_y[next_y.size()-2]; // also y
        double next_x_spl = spline_x(dl);                                      // next nonlinear x
        double next_y_spl = spline_y(dl);                                      // also y
        double d = 1e-9+distance(next_x_lin,next_y_lin,next_x_spl,next_y_spl); // d is the differenc between spline and linear path
        double d_pp = 2*d/(dt*dt);                 // second derivative of d: dpp = d/dt² ,which result in rotating acceleration
        double d_max = min( a_max, d_pp )/2*dt*dt; // d must be limmted by given a_max and calculated the maximal d(a_max)
        double d_ratio = d_max / d;              // d min = 1e-9 not 0, ratio to avoid sin and cos functions
        //std::cout << "d: " << d << " d_ratio: " << d_ratio << " d_pp: " << d_pp << std::endl;
        next_x.push_back( next_x_lin+(next_x_spl-next_x_lin)*d_ratio );
        next_y.push_back( next_y_lin+(next_y_spl-next_y_lin)*d_ratio );
        //next_x.push_back(next_x_spl);
        //next_y.push_back(next_y_spl);
    } 
 //std::cout << "######END Trajectory####################################### "<< std::endl;
}

