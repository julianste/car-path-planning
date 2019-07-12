#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#include "PathGenerator.h"
#include "Vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

void handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

int main() {
	signal(SIGSEGV, handler);   // install our handler
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<PathGenerator::landmark_s> landmarks;

  // Waypoint map to read from
  string map_file_ = "./data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    struct PathGenerator::landmark_s landmark = {x, y, s, d_x, d_y};
    landmarks.push_back(landmark);
  }
  PathGenerator pathGenerator(landmarks);


  vector<Vehicle> otherCars;

  h.onMessage([&pathGenerator]
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
          
          // Ego car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          // The data format for each car is: [ id, x, y, vx, vy, s, d].
          // The id is a unique identifier for that car.
          // The x, y values are in global map coordinates, and the vx, vy values are the velocity components,
          // also in reference to the global map. Finally s and d are the Frenet coordinates for that car.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          pathGenerator.updateOtherCars(sensor_fusion);
          pathGenerator.updateState(previous_path_x, previous_path_y, end_path_s, end_path_d, car_x, car_y, car_s, car_d, car_yaw, car_speed);

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          pathGenerator.get_next_vals(next_x_vals, next_y_vals);
          cout << "First x val: "<< next_x_vals[0] << endl;
          cout << "Second x val: "<< next_x_vals[1]<< endl;
          cout << "First y val: "<< next_y_vals[0]<< endl;
          cout << "Second y val: "<< next_y_vals[1]<< endl;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

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
