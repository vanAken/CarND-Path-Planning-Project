#include <iostream>
#include <cmath>
#include "trajectory.h"
#include "spline.h"

Trajectory::Trajectory( vector<double> previous_path_x, vector<double> previous_path_y, double dt, double v_max, double a_max, Frenet frenet)
{  
   long max_waypoints =800;
   next_x = previous_path_x;   // take the old path
   next_y = previous_path_y;
   while (next_x.size() > 30){ // and cut it down as needed for a* time 
        next_x.pop_back();
        next_y.pop_back();
    }
    // create first two spline points from previous_path
    spline_points_x = {next_x[next_x.size()-2],next_x[next_x.size()-1]};
    spline_points_y = {next_y[next_y.size()-2],next_y[next_y.size()-1]};
    double dl  = distance( spline_points_x[0],spline_points_y[0],
                           spline_points_x[1],spline_points_y[1]);
    double v_t = dl/dt; // initial velocity at the end of the path
    spline_points_v = {v_t, v_t};

    // create the next spline points from A*
    vector<double> next_xy; // tmp var
    vector<double> next_sd0;
    next_sd0 = frenet.xy_to_sd(spline_points_x[1],spline_points_y[1]);
        for(int i = 0; i < ::next_s.size(); i++) {
            if (next_sd0[0]+4 < ::next_s[i] || 6945. < ::next_s[i]){ // is next_s from A* still 4m in front of spline_points?
                next_xy = frenet.sd_to_xy(::next_s[i], ::next_d[i]);
                spline_points_x.push_back(next_xy[0]);
                spline_points_y.push_back(next_xy[1]);
                spline_points_v.push_back(::next_v[i]);
            //    next_xy = frenet.sd_to_xy(::next_s[i]+1, ::next_d[i]);
            //    spline_points_x.push_back(next_xy[0]);
            //    spline_points_y.push_back(next_xy[1]);
            //    spline_points_v.push_back(::next_v[i]);
            }
        }
    //for (int i=0; i < spline_points_x.size();i++){
      //  std::cout << " spline_points_x: " << spline_points_x[i] << 
        //             " spline_points_y: " << spline_points_y[i] << std::endl;
    //}
   //std::cout << "############################################# "<< std::endl;

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
        double next_x_spl = spline_x(dl);                                      // next nolinear x
        double next_y_spl = spline_y(dl);                                      // also y
        double d = 1e-9+distance(next_x_lin,next_y_lin,next_x_spl,next_y_spl); // d is the differenc between spline and linear path
        double d_pp = d/(dt*dt);                 // second derivative of d: dpp = d/dt² ,which result in rotating acceleration
        double d_max = min( a_max, d_pp )*dt*dt; // d must be limmted by given a_max and calculated the maximal d(a_max)
        double d_ratio = d_max / d;              // d min = 1e-9 not 0, ratio to avoid sin and cos functions
        //std::cout << "d: " << d << " d_ratio: " << d_ratio << " d_pp: " << d_pp << std::endl;
        //next_x.push_back(next_x_lin+(next_x_spl-next_x_lin)*d_ratio);
        //next_y.push_back(next_y_lin+(next_y_spl-next_y_lin)*d_ratio);
        next_x.push_back(next_x_spl);
        next_y.push_back(next_y_spl);
    } 
 //std::cout << "######END Trajectory####################################### "<< std::endl;
}
vector<double> Trajectory::next_X() {
    return next_x;
}
vector<double> Trajectory::next_Y() {
    return next_y;
}
