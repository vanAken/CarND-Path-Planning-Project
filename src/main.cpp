#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helpers.h"
#include "spline.h"
#include "frenet.h"
#include "trajectory.h"
#include "prediction.h"

int main() {
  uWS::Hub h;
  string map_file_ = "../data/highway_map.csv";
  Frenet frenet(map_file_);

  double v_max = 48.75 / 2.24   ;         // turn mph into m/s
  double a_max = 10.0;                    // m/s² 
  double time_counter_s = 999;            // a* counter is high to do a A* search first
  vector<double> next_s, next_d, next_v;  // store A* results          

  h.onMessage([&frenet, v_max, a_max, &time_counter_s, &next_s, &next_d, &next_v]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = nlohmann::json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
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

          const double dt   = 0.02; // telemetry loop
          const double d_dt = 1.0 ;  //    A*     loop

          nlohmann::json msgJson;
          string msg = "42[\"manual\",{}]";  // default value ==> Manual driving 

          // generate the first two points for v=0.5m/s, if previous_path_x is empty
          if(previous_path_x.size() < 2) { 
              double ini_l = max(car_speed,0.5) * dt; // l = v * t or 0.5 for low speed
              previous_path_x = {car_x, car_x + cos(deg2rad(car_yaw))* ini_l};
              previous_path_y = {car_y, car_y + sin(deg2rad(car_yaw))* ini_l};  
              msgJson["next_x"] = previous_path_x;    // send the initial points to telemetry
              msgJson["next_y"] = previous_path_y;   
              msg = "42[\"control\","+ msgJson.dump()+"]123";
          }  

          // either recalculate a* or calculate the trajectory
          time_counter_s += dt;              
          if (time_counter_s >= d_dt){  
              time_counter_s = 0;   // reset counter
              // Create an instance of Prediction and fill the global timeroad
              Prediction path (car_s, d_dt, sensor_fusion,previous_path_x, previous_path_y, frenet);
              path.search(car_s, car_d,car_speed/2.24, v_max, a_max, sensor_fusion, d_dt); //start A* 
              next_s = path.next_s;
              next_d = path.next_d;
              next_v = path.next_v;          
              print_time_raod(); // print the discrete solution to std_out
          } 
          else{
              Trajectory trajectory(previous_path_x, previous_path_y, v_max, a_max, frenet, next_s, next_d, next_v, sensor_fusion, dt);
              msgJson["next_x"] = trajectory.next_x;
              msgJson["next_y"] = trajectory.next_y;                            
              msg = "42[\"control\","+ msgJson.dump()+"]123";
          }
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);  
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
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
