#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>

using namespace std;

class Trajectory {

private:
    vector<double> next_x, next_y;
    vector<double> spline_points_x, spline_points_y, spline_points_v; 

public:
    Trajectory( vector<double> previous_path_x, vector<double> previous_path_y, double dt, double max_speed, double max_acc, Frenet frenet);
    vector<double> next_X();
    vector<double> next_Y();
};

#endif
