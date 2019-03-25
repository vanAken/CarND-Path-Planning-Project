#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>

using namespace std;

class Trajectory {

public:
    Trajectory( vector<double> previous_path_x, vector<double> previous_path_y, double max_speed, double max_acc, 
                Frenet frenet, vector<double>next_s, vector<double>next_d, vector<double>next_v, double dt);
    vector<double> next_X();
    vector<double> next_Y();

private:
    vector<double> next_x, next_y;
    vector<double> spline_points_x, spline_points_y, spline_points_v; 

};

#endif
