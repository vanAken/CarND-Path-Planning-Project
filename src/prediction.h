#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include <unordered_map>
#include <set>
#include <queue>
#include <iostream>
#include <string>
#include <fstream>

using namespace std;

//class MapSearchNode; // forward deklaration

class Prediction {

public:
    Prediction(double ego_s, double pre_t, vector<vector<double>> sensor_fusion);
    ~Prediction();

    void search (double ego_s, double ego_d, double ego_v, double v_max, double a_max, double d_dt);
    vector<double> path_s, path_d, path_v;

private:
    int      discrete2s (double s, double ego_s);
    double continuous2s (int    s, double ego_s);

    int      discrete2d (double d);
    double continuous2d (int    d);

    double   discrete2v (double v, double pre_t);
    double continuous2v (double v, double pre_t);
};

#endif

