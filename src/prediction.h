#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include <unordered_map>
#include <set>
#include <queue>

using namespace std;

class Prediction {
public:
    Prediction(double ego_s,double ego_d,double ego_v,vector<vector<double>> sensor_fusion);
    ~Prediction();
    int GetMap( int d, int s, int t );
    void search ();
    vector<double> path_s, path_d, path_v;
private:
    double _ego_s, _ego_d, _ego_v;
    int discrete, num_of_lanes, rearview;
    int d_horizont_s,d_horizont_t;
    int d_ego_s, d_ego_d, d_ego_v;
    vector<int> time_road;
    int discrete2s (double s);
    int discrete2d (double d);
    int discrete2v (double v);
    double continuous2s (int s);
    double continuous2d (int d);
    double continuous2v (int v);
};

#endif

