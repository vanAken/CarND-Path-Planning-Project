#ifndef PATH_PLANNING_PREDICTION_H
#define PATH_PLANNING_PREDICTION_H

#include <unordered_map>
#include <set>
#include <queue>

using namespace std;

//class MapSearchNode; // forward deklaration

class Prediction {

private:
    double _ego_s, _ego_d, _ego_v;
    const int discrete =  4;
    const int rearview = 40;
    int d_ego_s, d_ego_d, d_ego_v;
    int time_road[5400]; //num_of_lanes * d_horizont_s * d_horizont_t
    int discrete2s (double s);
    int discrete2d (double d);
    int discrete2v (double v);
    double continuous2s (int s);
    double continuous2d (int d);
    double continuous2v (int v);

public:
    Prediction(double ego_s,double ego_d,double ego_v,vector<vector<double>> sensor_fusion);
    ~Prediction();

    int GetMap( int d, int s, int t );
    const int num_of_lanes =  3;
    const int d_horizont_s =(80+rearview) / discrete;
    const int d_horizont_t = 60;

    void search ();
    vector<double> path_s, path_d, path_v;
};

#endif

