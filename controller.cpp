#include <iostream>
#include <cmath>
#include "controller.h"
#include "spline.h"

Controller::Controller(vector<double> car_x, vector<double> car_y, double dt,
                       vector<double> traj_x, vector<double> traj_y, vector<double> traj_v,
                       double pred_time)
{   next_x = {car_x[0],car_x[1]};
    next_y = {car_y[0],car_y[1]};

    double minv = 0.0015 / 0.02; // Ensure at least 0.01 meters per tick to avoid rounding problems.
    double car_s = distance(car_x[0],car_y[0],car_x[1],car_y[1]);
    double v = max(minv, car_s/ dt); // initial velocity
    int max_waypoints = 200;

    // create spline point
    vector<double> spline_points_x = next_x;
    vector<double> spline_points_y = next_y;
    for(int i = 0; i < traj_x.size(); i++) {
        double t = pred_time + i;
        if(t > 2 *dt) {
            spline_points_x.push_back(traj_x[i]);
            spline_points_y.push_back(traj_y[i]);
        }
    }
    // frenet s is used as the spline x value
    double s = 0.0, x = car_x[0], y = car_y[0]; // car position
    vector<double> spline_points_s;
    for(int i = 0; i < spline_points_x.size(); i++) {
        s += distance(x, y, spline_points_x[i],spline_points_y[i]);
        spline_points_s.push_back(s);
        x = spline_points_x[i];
        y = spline_points_y[i];
    }
    tk::spline spline_x;
    tk::spline spline_y;
    spline_x.set_points(spline_points_s,spline_points_x);
    spline_y.set_points(spline_points_s,spline_points_y);   
    while(next_x.size() < max_waypoints) {
        double t = 1 + next_x.size() * dt - pred_time;
        double traj1 = max(0, int(t));
        double traj2 = traj1 + 1;
        double traj_weight2 = t - traj1;
        double traj_weight1 = 1 - traj_weight2;
        double traj_plus_v = traj_v[traj1] * traj_weight1 + traj_v[traj2] * traj_weight2;
        v = max(minv,v + (traj_plus_v - v) * dt);
        car_s += v * dt;
        next_x.push_back(spline_x(car_s));
        next_y.push_back(spline_y(car_s));
    }
}
vector<double> Controller::next_X() {
    return next_x;
}
vector<double> Controller::next_Y() {
    return next_y;
}
