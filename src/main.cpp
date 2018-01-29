#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

#include "trajectory_generator.h"
#include "predictor.h"
#include "vehicle.h"
#include "helpers.h"
#include "road.h"
#include "constants.h"


using namespace std;

// for convenience
using json = nlohmann::json;
using Eigen::MatrixXd;
using Eigen::VectorXd;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // create road object using map waypoints
  Road road = Road(COMMON_SPEED_LIMIT, COMMON_LANE_WIDTH, COMMON_N_LANES, COMMON_MAX_S,
                   map_waypoints_x, map_waypoints_y,
                   map_waypoints_s, map_waypoints_dx,
                   map_waypoints_dy);

  // initialize predictor
  Predictor predictor = Predictor(road);

  // init trajectory generator
  TrajectoryGenerator traj_gen = TrajectoryGenerator(road, predictor);

  h.onMessage([&road,&predictor,&traj_gen](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          car_yaw = deg2rad(car_yaw);
          car_speed = car_speed / 2.237; // convert to m/s

//          cout << "telemetry: x, y, s, d, yaw, speed = " << car_x << ", " << car_y << ", "
//               << car_s << ", " << car_d << ", " << car_yaw << ", " << car_speed << endl;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          // Always contains 12 vehicles with ids 0 to 11.
          vector< vector<double> > sensor_fusion = j[1]["sensor_fusion"];

          // update targets
          predictor.update_targets(sensor_fusion);

          // update host - TODO: poor class isolation/definition - wouldn't host.update() be cleaner?
          vector<double> host_frenet_state;
          vector<double> host_xy;
          int prev_size;
          if (predictor.host_init_) {
            prev_size = previous_path_x.size();
            host_frenet_state = traj_gen.get_host_state(prev_size);
            host_xy = traj_gen.get_host_xy(prev_size);
          } else {
            prev_size = 50;
            host_frenet_state = {car_s, 0, 0, car_d, 0, 0};
            host_xy = {car_x, car_y};
          }

          predictor.update_host(host_frenet_state, host_xy[0], host_xy[1], car_yaw);
//          predictor.update_lane_selector(4.0);
//          predictor.host_.print();

//          vector< vector<double> > next_points = traj_gen.keep_lane(predictor.desired_lane_, 4.0, prev_size);
          vector< vector<double> > next_points = traj_gen.keep_lane(3.0, prev_size);
          vector<double> next_x_vals = next_points[0];
          vector<double> next_y_vals = next_points[1];

          double max_delta_dist = 0;
          for (int i = 0; i < next_x_vals.size() - 1; i++) {
            double delta_dist = distance_2d(next_x_vals[i], next_y_vals[i], next_x_vals[i+1], next_y_vals[i+1]);
            if (delta_dist > max_delta_dist) {
              max_delta_dist = delta_dist;
            }
          }
//          cout << "main: max_speed = " << 2.237 * max_delta_dist / TRAJECTORY_TIME_STEP << endl;

//          cout << "main: next x[0], y[0], x[end], y[end] = " << next_x_vals[0] << ", " << next_y_vals[0] << ", "
//               << next_x_vals[next_x_vals.size() - 1] << ", " << next_y_vals[next_y_vals.size() - 1] << endl;
//          cout << "speed, yaw, prev_size = " << car_speed << ", " << car_yaw << ", " << prev_size << endl;
//
//          cout << "telem    s, d, x, y: " << car_s << ", " << car_d << ", " << car_x << ", " << car_y << endl;
//          cout << "traj_gen s, d, x, y: " << host_frenet_state[0] << ", " << host_frenet_state[3] << ", " << host_xy[0] << ", " << host_xy[1] << endl;
//
//          cout << "dx: ";
//          if (traj_gen.prev_pts_s_.size() > 0) {
//            for (int i = 0; i < 5; i++) {
//              double dx = traj_gen.prev_pts_x_[i+1] - traj_gen.prev_pts_x_[i];
//              cout << dx << ", ";
//            }
//            cout << endl;
//            cout << "dy: ";
//            for (int i = 0; i < 5; i++) {
//              double dy = traj_gen.prev_pts_y_[i+1] - traj_gen.prev_pts_y_[i];
//              cout << dy << ", ";
//            }
//            cout << endl;
//          }
//          cout << "ds: ";
//          if (traj_gen.prev_pts_s_.size() > 0) {
//            for (int i = 0; i < 5; i++) {
//              double ds = traj_gen.prev_pts_s_[i+1] - traj_gen.prev_pts_s_[i];
//              cout << ds << ", ";
//            }
//            cout << endl;
//            cout << "dd: ";
//            for (int i = 0; i < 5; i++) {
//              double dd = traj_gen.prev_pts_d_[i+1] - traj_gen.prev_pts_d_[i];
//              cout << dd << ", ";
//            }
//            cout << endl;
//          }

          // assemble message
          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
  size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
