#ifndef PATH_PLANNING_PID_CONTROLLER_H
#define PATH_PLANNING_PID_CONTROLLER_H

#include <vector>

using namespace std;

class Controller {
private:
    vector<double> next_x, next_y;
public:
    Controller(vector<double> car_x, vector<double> car_y, double dt, double max_speed,
               vector<double> traj_x, vector<double> traj_y, vector<double> traj_v,
               double pred_time);
    vector<double> next_X();
    vector<double> next_Y();
};

#endif
