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
#include "frenet.h"
#include "frenet.cpp"
#include "controller.h"
#include "controller.cpp"
#include "prediction.h"
#include "prediction.cpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::ifstream;

int main() {
  uWS::Hub h;

  string map_file_ = "../data/highway_map.csv";
  Frenet frenet(map_file_);

  double max_speed = 39.0 / 2.24   ; // turn mph into m/s
  double max_acc   = 9.0;           // m/s²          
  h.onMessage([&frenet, &max_speed, &max_acc]
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
          double dt = 0.02;

          // Create an instance of Prediction and Initialize it  
          Prediction trajectory (car_s, car_d, car_speed/2.24, sensor_fusion);

          // start A*  
          trajectory.search(); 

          vector<double> next_s = {car_s+4,car_s+8,car_s+16,car_s+32,car_s+64,car_s+128};
          vector<double> next_d = {      6,      6,       6,       6,       6,        6};
          vector<double> next_v = {     20,     20,      20,      20,      20,       20};       
   
          // transform back inte global CS
          vector<double> next_x(0); 
          vector<double> next_y(0); 
          for (long i = 0; i < next_s.size(); i++) {
              vector<double> next_xy = frenet.sd_to_xy(next_s[i], next_d[i]);
              next_x.push_back(next_xy[0]);
              next_y.push_back(next_xy[1]);
          }

          // generate the first two points for v=0.5m/s if previous_path_x is too empty
          if(previous_path_x.size() < 2) { 
             double ini_l = max(.5, car_speed) * dt; // l = v * t or 0.5 for low speed
             previous_path_x = {car_x, car_x + cos(deg2rad(car_yaw))* ini_l};
             previous_path_y = {car_y, car_y + sin(deg2rad(car_yaw))* ini_l};
          }  

          // generate the dot for the packman give the controller freenet ?
          Controller dots(previous_path_x, previous_path_y, dt, max_speed,
                         next_x, next_y , next_v);
          json msgJson;
          msgJson["next_x"] = dots.next_X();
          msgJson["next_y"] = dots.next_Y();                
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
