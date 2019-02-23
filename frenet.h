#ifndef PATH_PLANNING_FRENET_H
#define PATH_PLANNING_FRENET_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include "spline.h"

using namespace std;

class Frenet {
public:
    explicit Frenet(string map_file);
    Frenet(const Frenet &frenet);
    virtual ~Frenet();

    // Transform Frenet s, d coordinates into Cartesian x, y
    vector<double> sd_to_xy(double s, double d);

    // Transform Cartesian x, y into Frenet s, d
    vector<double> xy_to_sd(double x, double y);

    // Transform Frenet s, d, vs, vd coordinates into Cartesian x, y, vx, vy
    vector<double> sdv_to_xyv(double s, double d, double vs, double vd);

    // Transform Cartesian x, y, vx, vy to Frenet s, d, vs, vd
    vector<double> xyv_to_sdv(double x, double y, double vx, double vy);

private:
    tk::spline s_x;
    tk::spline s_y;
    tk::spline s_dx;
    tk::spline s_dy;
    bool circular;
    double min_waypoint_s;
    double max_waypoint_s;
    double track_s;
};

#endif
