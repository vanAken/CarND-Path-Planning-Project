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
#include "trajectory_planner.h"
#include "trajectory_planner.cpp"
#include "controller.h"
#include "controller.cpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::ifstream;

int main() {
  uWS::Hub h;

  string map_file_ = "../data/highway_map.csv";
  Track track(map_file_);

  double max_speed = 45. / 2.24   ; // turn mph into m/s
  double max_acc   = 9.5;           // m/s²          
  h.onMessage([&track, &max_speed, &max_acc]
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
          double dt = 0.02;

          vector<double> cars_s(0), cars_d(0), cars_vs(0), cars_vd(0);
          for(int i = 0; i < sensor_fusion.size(); i++) {
              double d = sensor_fusion[i][6];
              double sens_x = sensor_fusion[i][1];
              double sens_y = sensor_fusion[i][2];
              double sens_vx = sensor_fusion[i][3];
              double sens_vy = sensor_fusion[i][4];
              vector<double> sens_sdv = track.xyv_to_sdv(sens_x,sens_y,sens_vx,sens_vy);
              cars_s.push_back(sens_sdv[0]);
              cars_d.push_back(sens_sdv[1]);
              cars_vs.push_back(sens_sdv[2]);
              cars_vd.push_back(sens_sdv[3]);
          }

          // cut down the previous_path for prediction
          while(previous_path_x.size() > 50) {
              previous_path_x.pop_back();
              previous_path_y.pop_back();
          }  
          // Predict the position of the other cars at the end of the path TODO Jerky cars
          double pred_time = previous_path_x.size() * dt;
          vector<double> pred_cars_s, pred_cars_d;
          for(int i = 0; i < cars_s.size(); i++) {
              pred_cars_s.push_back(cars_s[i] + pred_time * cars_vs[i]);
              pred_cars_d.push_back(cars_d[i] + pred_time * cars_vd[i]);
          }
         
          // generate the first two points for v=0.5m/s
          if(previous_path_x.size() < 2) { 
             previous_path_x = {car_x, car_x + cos(deg2rad(car_yaw))*.01};
             previous_path_y = {car_y, car_y + sin(deg2rad(car_yaw))*.01};
          }  
  
          // get the two last points of the previous path for the planner
          double eop_x =     previous_path_x[previous_path_x.size()-1];
          double eop_y =     previous_path_y[previous_path_x.size()-1];
          double eop_x_pre = previous_path_x[previous_path_x.size()-2];
          double eop_y_pre = previous_path_y[previous_path_x.size()-2];
          double eop_vx = (eop_x - eop_x_pre) / dt;
          double eop_vy = (eop_y - eop_y_pre) / dt;
          // transform them into frenet CS
          vector<double> end_path = track.xyv_to_sdv(eop_x, eop_y, eop_vx, eop_vy);
          // initialize the planner
          double time_horizont = 10.;
          TrajectoryPlanner planner(end_path[0], end_path[1], end_path[2], 0.0,
                                    pred_cars_s, pred_cars_d, cars_vs, cars_vd,
                                    max_speed, max_acc, time_horizont);
          planner.calculate(0.01);
          vector<double> next_s = planner.pathS();
          vector<double> next_d = planner.pathD();
          vector<double> next_v = planner.pathV();          
          // transform back inte global CS
          vector<double> next_x(0); 
          vector<double> next_y(0); 
          for (long i = 0; i < next_s.size(); i++) {
              vector<double> next_xy = track.sd_to_xy(next_s[i], next_d[i]);
              next_x.push_back(next_xy[0]);
              next_y.push_back(next_xy[1]);
          }
          
          Controller pid(previous_path_x, previous_path_y, dt,
                         next_x, next_y , next_v, pred_time);
          json msgJson;
          msgJson["next_x"] = pid.next_X();
          msgJson["next_y"] = pid.next_Y();                
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
