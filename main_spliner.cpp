#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "track.h"
#include "track.cpp"
//#include "trajectory_planner.h"
//#include "trajectory_planner.cpp"
//#include "controller.h"
//#include "controller.cpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::ifstream;

int main() {
  uWS::Hub h;

  string map_file_ = "../data/highway_map.csv";
  Track track(map_file_);

  double speed_limet = 45./ 2.24   ; // turn mph into m/s
  double speed = speed_limet;
  int lane = 2;
  
  h.onMessage([&track, &speed_limet, &speed, &lane]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
 
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          
          /**
          * TODO: define a path made up of (x,y) points that the car will visit
          *   sequentially every .02 seconds
          */
  
          // avoid hight accelation from zero speed  
          if (car_speed == 0.0 ) speed = 0.5;
          if (speed < speed_limet) speed += .5;         
          //std::cout << "speed " << speed <<" car_speed "<< car_speed << std::endl;
        
          vector<double> next_x_vals; // to be defined
          vector<double> next_y_vals; // path in Golbal CS 
          double next_s;              // result of the planner 
          double next_d;              // path poiont in frenet
              
          // cut down the previous_path and copy this into the next values 
          while(previous_path_x.size() > 50) {
             previous_path_x.pop_back();
             previous_path_y.pop_back();
          }  
          for (int i = 0; i < previous_path_x.size(); i++) {
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
          }

          // Initialize the Trajectory
          if(previous_path_x.size() < 2) { // make shure previous_path has two members
             previous_path_x = {car_x - cos(deg2rad(car_yaw)), car_x };
             previous_path_y = {car_y - sin(deg2rad(car_yaw)), car_y };
          }
          // get the two last points of the previous path for the new path
          double eop_x =     previous_path_x[previous_path_x.size()-1];
          double eop_y =     previous_path_y[previous_path_x.size()-1];
          double eop_x_pre = previous_path_x[previous_path_x.size()-2];
          double eop_y_pre = previous_path_y[previous_path_x.size()-2];
          double eop_vx = (eop_x - eop_x_pre) / 0.02;
          double eop_vy = (eop_y - eop_y_pre) / 0.02;
          // transform them into frenet 
          vector<double> end_path = track.xyv_to_sdv(eop_x, eop_y, eop_vx, eop_vy);
          vector<double> end_path_pre = track.xy_to_sd(eop_x_pre, eop_y_pre);
          
          // set lane to periodic, but a* will do that later
          std::cout <<"start new lane  ====================="<< std::endl;
          int new_lane = lane;
          if (end_path[0] > 300) new_lane = 1+ (static_cast<int>(end_path[0]/150)) % 3;
          std::cout <<"new_lane "<< new_lane << " lane " << lane << std::endl;

          
              vector<double>ptsx;
              vector<double>ptsy;
              double end_path_s = end_path[0] + end_path[2]*1.; // new s s0 + v0(*1s) += 50 points
              int    end_path_d = new_lane*4-2; // new d laneposition in the middle of the lane  
              ptsx.push_back(end_path_pre[0]);
              ptsy.push_back(end_path_pre[1]);
              ptsx.push_back(end_path[0]);
              ptsy.push_back(end_path[1]);
              ptsx.push_back(end_path_s);
              ptsy.push_back(end_path_d);
              ptsx.push_back(end_path_s+.1);
              ptsy.push_back(end_path_d);
              ptsx.push_back(end_path_s+1);
              ptsy.push_back(end_path_d);
              //ptsx.push_back(end_path_s+3);
              //ptsy.push_back(end_path_d);              
              tk::spline trajectory;     // inilize path dot
              //double path_dot = end_path[1]-end_path_pre[1]/end_path[0]-end_path_pre[0]; // dd/ds @end path
              //trajectory.set_boundary(tk::spline::first_deriv,path_dot,tk::spline::second_deriv,0.0,false);
              //trajectory.set_boundary(tk::spline::second_deriv,0.0,tk::spline::first_deriv,0.0,false);
	      trajectory.set_points(ptsx,ptsy);
              for (long i = 1; i <= 400-next_x_vals.size(); i++) {
                  next_s = end_path[0] + i * 0.02 * speed;
                  next_d = trajectory(next_s); 
                  vector<double> next_xy = track.sd_to_xy(next_s, next_d);
                  next_x_vals.push_back(next_xy[0]);
                  next_y_vals.push_back(next_xy[1]);
                  // output
                  //std::cout << next_s <<" "<< next_d <<" "<< end_path[0] << std::endl;
              }
              // finalize lane change
              std::cout << next_d  << " lane " << lane  << end_path[1]<< std::endl;
              if (abs(next_d-end_path[1]) < .01) lane = new_lane; 
              end_path = {next_s,next_d}; //update end of path 
              std::cout <<"end new lane  ====================="<< std::endl;
           
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]123";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
