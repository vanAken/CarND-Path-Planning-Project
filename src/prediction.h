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

    vector<double> next_S();
    vector<double> next_D();
    vector<double> next_V();

private:
    vector<double> next_s, next_d, next_v;

    int      discrete_to_s (double s, double ego_s);
    double continuous_to_s (int    s, double ego_s);

    int      discrete_to_d (double d);
    double continuous_to_d (int    d);

    int      discrete_to_v (double v, double pre_t);
    double continuous_to_v (double v, double v_max, double pre_t);
};

#endif

