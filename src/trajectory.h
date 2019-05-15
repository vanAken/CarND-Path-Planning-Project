#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

class Trajectory {

public:
    Trajectory( vector<double> previous_path_x, vector<double> previous_path_y, double v_max, double a_max, 
                Frenet frenet, vector<double>next_s, vector<double>next_d, vector<double>next_v, vector<vector<double>> sensor_fusion, double dt);

    vector<double> next_x, next_y;

private:
    vector<double> spline_points_x, spline_points_y, spline_points_v, spline_points_s;

    vector<double> calc_gap( double d, double ego_s, double ego_v, vector<vector<double>>sensor_fusion );
};

// ---------------------------------------------------------------------
// implementation part, which could be separated into a cpp file
// ---------------------------------------------------------------------


Trajectory::Trajectory( vector<double> previous_path_x, vector<double> previous_path_y, double v_max, double a_max,
                        Frenet frenet, vector<double>next_s, vector<double>next_d, vector<double>next_v, vector<vector<double>> sensor_fusion, double dt)
{   const long max_waypoints = 999;
    next_x = {previous_path_x[0],previous_path_x[1]};   // take the first two points of the old path
    next_y = {previous_path_y[0],previous_path_y[1]};   // as start points for the next trajectory
    double dl  = distance( next_x[0],next_y[0],next_x[1],next_y[1]); // between the initial points 
    double v_t = dl/dt; // initial velocity = initial distance / time increment

    // create first two spline points from the beginning of the previous_path
    spline_points_x = next_x;
    spline_points_y = next_y;
    spline_points_v = {v_t, v_t};
    spline_points_s = {0  , dl };

    vector<double> next_sd0; // calculate s and d from last spline_points
    next_sd0 = frenet.xy_to_sd(spline_points_x[1],spline_points_y[1]);
  
    //calcuate average d of d values from A* 
    next_d[0] = continuous_to_d(discrete_to_d( (next_d[1]+next_d[2]+next_d[3])/3 ));
    vector<double> lane_gap = {0,0,0,0}; // tmp var 
    if (    next_d[0]+2 < next_sd0[1]) { // left lane change => free of other cars?
        if (next_d[0]+6 < next_sd0[1]) { // only one lane max 
            next_d[0] += 4;  
            std::cout << "############### LEFT ######################" << std::endl;
        }
        lane_gap = Trajectory::calc_gap( next_d[0], next_sd0[0], v_t, sensor_fusion);
    }
    else if(next_d[0]-2 > next_sd0[1]) { // right lane change => free of other cars?
        if (next_d[0]-6 > next_sd0[1]){
            next_d[0] -= 4;  // only one lane max
            std::cout << "############### RIGHT #####################" << std::endl;
        }
        lane_gap = Trajectory::calc_gap( next_d[0], next_sd0[0], v_t, sensor_fusion);
    }
    if ( -10 < lane_gap[1] || lane_gap[0] < 10 ){ // if gap to small ==> follower mode
         next_d[0] = continuous_to_d(discrete_to_d(next_sd0[1]));
         next_s[0] = next_sd0[0]+v_t; // avoid out of lane by
    }
    else{ 
        next_s[0] = next_sd0[0]+v_t/10;     // skipp this point during lane change
    }
    next_s[1] = next_sd0[0]+2*v_t;    // change next_s in a defined position
    next_s[2] = next_sd0[0]+2.5*v_t;  
    next_s[3] = next_sd0[0]+3*v_t;  
    next_d[1] = next_d[0];            // copy average d 
    next_d[2] = next_d[0];
    next_d[3] = next_d[0];

    lane_gap = Trajectory::calc_gap( next_d[0], next_sd0[0], v_t, sensor_fusion);
    if(lane_gap[0]  < v_max ){      //drive slower than the others cars speed if close
        next_v[0] = lane_gap[2]-3;
        next_v[1] = next_v[0];
        next_v[2] = next_v[0];
        next_v[3] = next_v[0];
    }

    // push the next spline points from A* 
    vector<double> next_xy; // tmp var
    double pre_next_s = next_sd0[0]; 
    for(int i = 1; i < next_s.size(); i++) {
        if (pre_next_s+v_t/5 < next_s[i]){  // push only if next_s is rising 
            next_xy = frenet.sd_to_xy( next_s[i], next_d[i]);
            spline_points_x.push_back( next_xy[0] );
            spline_points_y.push_back( next_xy[1] );
            spline_points_v.push_back( next_v[i]  );
            next_xy = frenet.sd_to_xy( next_s[i]+v_t/10, next_d[i]); 
            spline_points_x.push_back( next_xy[0] ); // double points to avoid overshoot
            spline_points_y.push_back( next_xy[1] );
            spline_points_v.push_back( next_v[i]  ); 
            pre_next_s = next_s[i]; 
        }
    }
    // frenet s is used as the spline x value
    double s_spline = 0.0;
    for(int i = 2; i < spline_points_x.size(); i++) {
        s_spline += distance(spline_points_x[i-1],spline_points_y[i-1],
                             spline_points_x[i],spline_points_y[i]);
        spline_points_s.push_back(s_spline);
    }
    //generate splines
    tk::spline spline_x; 
    tk::spline spline_y;
    tk::spline spline_v;
    spline_x.set_points( spline_points_s, spline_points_x );
    spline_y.set_points( spline_points_s, spline_points_y );
    spline_v.set_points( spline_points_s, spline_points_v );      

    // generate waypoints
    while( next_x.size() < max_waypoints) {  
        if (v_t <= spline_v(v_t+dl) ) v_t = std::min(v_max, v_t + a_max*1/2 * dt); // future v
        else                          v_t = std::max( 0.  , v_t - a_max*3/4 * dt);
        //std::cout << "spline_v(32+dl): " << spline_v(32+dl) << "v_t: " << v_t << std::endl;
        dl += v_t * dt;
        double next_x_lin = 2*next_x[next_x.size()-1]-next_x[next_x.size()-2]; // next_x_lin = 2*x_2-x_1
        double next_y_lin = 2*next_y[next_y.size()-1]-next_y[next_y.size()-2]; // also y
        double next_x_spl = spline_x(dl);                                      // next nonlinear x
        double next_y_spl = spline_y(dl);                                      // also y
        double d = 1e-9+distance(next_x_lin,next_y_lin,next_x_spl,next_y_spl); // d is the differenc between spline and linear tangetial path normal to s
        double d_pp = 2*d/(dt*dt);                 // second derivative of d: dpp = d/dt² ,which result in lateral acceleration
        double d_max = std::min( a_max, d_pp )*dt*dt/2; // d must be limited by a_max or d_pp
        double d_ratio = d_max / d;             // the factor d_ratio is used to scals down d
        //std::cout << "d: " << d << " d_ratio: " << d_ratio << " d_pp: " << d_pp << std::endl;
        if (dl < v_t*2 ){ // apply d_ratio on the spline pints
            next_x.push_back( next_x_lin+(next_x_spl-next_x_lin)*d_ratio );
            next_y.push_back( next_y_lin+(next_y_spl-next_y_lin)*d_ratio );
        }
        else{   
            next_x.push_back(next_x_spl);
            next_y.push_back(next_y_spl);
        }
    } 
}

// calculate the front and rear gap of the ego car
vector<double> Trajectory::calc_gap( double d, double ego_s, double ego_v, vector<vector<double>>sensor_fusion ){
     double gap_s_front = 999;
     double gap_v_front = 25.;
     double gap_s_rear  =-999;
     double gap_v_rear  = 0;
     for(int i = 0; i < sensor_fusion.size(); i++){
     	// other car has same lane than ego car
       	if(::discrete_to_d(d) == ::discrete_to_d(sensor_fusion[i][6]) ){
	    double other_vx = sensor_fusion[i][3];
       	    double other_vy = sensor_fusion[i][4];
       	    double other_v  = sqrt(other_vx*other_vx+other_vy*other_vy);
       	    double other_s  = sensor_fusion[i][5];
            double distance = other_s - ego_s; 
       	    if(distance > 0.){ // is other car is in front?
                if(distance < +gap_s_front){ // new smalest distance?
		    gap_s_front = distance;
                    gap_v_front = other_v;
                }
            }
            else {// is other car is behind?
                if(distance > gap_s_rear){ // new smalest distance?
		    gap_s_rear = distance;
                    gap_v_rear = other_v;
                }
            } 
        }
    } 
    return {gap_s_front, gap_s_rear, gap_v_front, gap_v_rear};
}
#endif
