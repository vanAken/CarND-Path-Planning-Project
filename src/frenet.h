#ifndef PATH_PLANNING_FRENET_H
#define PATH_PLANNING_FRENET_H

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

// ---------------------------------------------------------------------
// implementation part, which could be separated into a cpp file
// ---------------------------------------------------------------------

Frenet::Frenet(string map_file) {
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    std::ifstream in_map(map_file.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map, line)) {
        std::istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        waypoints_x.push_back(x);
        waypoints_y.push_back(y);
        waypoints_s.push_back(s);
        waypoints_dx.push_back(d_x);
        waypoints_dy.push_back(d_y);
    }

    // close the loop if the endpoints are nearby
    min_waypoint_s = waypoints_s[0];
    max_waypoint_s = waypoints_s[waypoints_s.size()-1];
    double distance_to_close = distance( waypoints_x[0], waypoints_y[0], waypoints_x[waypoints_x.size()-1], waypoints_y[waypoints_y.size()-1]);
    circular = (distance_to_close < 100);

    // The circular need to add beginning points to end to create an overlap closed track
    vector<double> aug_x;
    vector<double> aug_y;
    vector<double> aug_s;
    vector<double> aug_dx;
    vector<double> aug_dy;
    track_s = max_waypoint_s - min_waypoint_s;
    if(circular) {
        track_s += distance_to_close;
        for(int i = waypoints_s.size() - 2; i < waypoints_s.size(); i++ ) {
            aug_x.push_back(waypoints_x[i]);
            aug_y.push_back(waypoints_y[i]);
            aug_s.push_back(waypoints_s[i] - track_s);
            aug_dx.push_back(waypoints_dx[i]);
            aug_dy.push_back(waypoints_dy[i]);
        }
    }
    for(int i = 0; i < waypoints_s.size(); i++) {
        aug_x.push_back(waypoints_x[i]);
        aug_y.push_back(waypoints_y[i]);
        aug_s.push_back(waypoints_s[i]);
        aug_dx.push_back(waypoints_dx[i]);
        aug_dy.push_back(waypoints_dy[i]);
    }
    if(circular) {
        for(int i = 0; i < 10; i++) {
            aug_x.push_back(waypoints_x[i]);
            aug_y.push_back(waypoints_y[i]);
            aug_s.push_back(waypoints_s[i] + track_s);
            aug_dx.push_back(waypoints_dx[i]);
            aug_dy.push_back(waypoints_dy[i]);
        }
    }

    // Set all waypoints to the spline 
    s_x.set_points(aug_s,aug_x);
    s_y.set_points(aug_s,aug_y);
    s_dx.set_points(aug_s,aug_dx);
    s_dy.set_points(aug_s,aug_dy);
}

Frenet::Frenet(const Frenet &frenet) {
    this->circular = frenet.circular;
    this->min_waypoint_s = frenet.min_waypoint_s;
    this->max_waypoint_s = frenet.max_waypoint_s;
    this->track_s = frenet.track_s;
    this->s_x = frenet.s_x;
    this->s_y = frenet.s_y;
    this->s_dx = frenet.s_dx;
    this->s_dy = frenet.s_dy;
}

Frenet::~Frenet() {}

// Transform Frenet s,d coordinates into Cartesian x,y
vector<double> Frenet::sd_to_xy(double s, double d) {
    double path_x = s_x(s);
    double path_y = s_y(s);
    double dx = s_dx(s);
    double dy = s_dy(s);
    double x = path_x + d * dx;
    double y = path_y + d * dy;
    return {x,y};
}

// Transform Cartesian x,y into Frenet s,d
vector<double> Frenet::xy_to_sd(double x, double y) {
    double s_est = min_waypoint_s;
    double dist2_est = pow(s_x(s_est) - x, 2) + pow(s_y(s_est) - y, 2);
    for(int i = 1; i <= 20; i++) {
        double s = i * (max_waypoint_s - min_waypoint_s) / 20 + min_waypoint_s;
        double dist2 = pow(s_x(s) - x, 2) + pow(s_y(s) - y, 2);
        if(dist2 < dist2_est) {
            dist2_est = dist2;
            s_est = s;
        }
    }
    for(int i = 0; i < 20; i++) {
        double x_est = s_x(s_est);
        double y_est = s_y(s_est);
        double x_diff = x - x_est;
        double y_diff = y - y_est;
        double dx = s_dx(s_est);
        double dy = s_dy(s_est);
        double plus_s_x = 0 - dy;
        double plus_s_y = dx;
        double plus_s_mag = distance(0,0,plus_s_x,plus_s_y);
        plus_s_x = plus_s_x / plus_s_mag;
        plus_s_y = plus_s_y / plus_s_mag;
        double delta_s = x_diff * plus_s_x + y_diff * plus_s_y;
        s_est += delta_s;
        if(abs(delta_s) < 0.01) break;
    }
    double x_est = s_x(s_est);
    double y_est = s_y(s_est);
    double x_diff = x - x_est;
    double y_diff = y - y_est;
    double dx = s_dx(s_est);
    double dy = s_dy(s_est);
    double d_mag = distance(0,0,dx,dy);
    dx = dx / d_mag;
    dy = dy / d_mag;
    double d_est = dx * x_diff + dy * y_diff;
    return {s_est,d_est};
}

// Transform Frenet s,d,vs,vd coordinates into Cartesian x,y,vx,vy
vector<double> Frenet::sdv_to_xyv(double s, double d, double vs, double vd) {
    double tiny = 0.01;
    vector<double> xy1 = sd_to_xy(s,d);
    double s2 = s + tiny * vs;
    double d2 = d + tiny * vd;
    vector<double> xy2 = sd_to_xy(s2,d2);
    double vx = (xy2[0] - xy1[0]) * (1 / tiny);
    double vy = (xy2[1] - xy1[1]) * (1 / tiny);
    return {xy1[0],xy1[1], vx, vy};
}

// Transform Cartesian x,y,vx,vy into Frenet s,d,vs,vd
vector<double> Frenet::xyv_to_sdv(double x, double y, double vx, double vy) {
    double tiny = 0.01;
    vector<double> sd1 = xy_to_sd(x,y);
    double x2 = x + tiny * vx;
    double y2 = y + tiny * vy;
    vector<double> sd2 = xy_to_sd(x2,y2);
    double vs = (sd2[0] - sd1[0]) * (1 / tiny);
    double vd = (sd2[1] - sd1[1]) * (1 / tiny);
    return {sd1[0], sd1[1], vs, vd};
}

#endif
