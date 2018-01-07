#include "road.h"

#include <iostream>

#include "helpers.h"
#include "constants.h"


using namespace std;

Road::Road(double speed_limit, double lane_width, int n_lanes, double max_s,
           vector<double>maps_x, vector<double>maps_y, vector<double>maps_s,
           vector<double>maps_dx, vector<double>maps_dy) {
  speed_limit_ = speed_limit;
  lane_width_ = lane_width;
  n_lanes_ = n_lanes;
  max_s_ = max_s;
  maps_x_ = maps_x;
  maps_y_ = maps_y;
  maps_s_ = maps_s;
  maps_dx_ = maps_dx;
  maps_dy_ = maps_dy;

  // create splines for get_xy
  s_x_spline_.set_points(maps_s_, maps_x_);
  s_y_spline_.set_points(maps_s_, maps_y_);
  s_dx_spline_.set_points(maps_s_, maps_dx_);
  s_dy_spline_.set_points(maps_s_, maps_dy_);
}


Road::~Road() {}


int Road::closest_waypoint(double x, double y) const {
  double closest_len = 100000; //large number
  int closest_wp = 0;

  for(int i = 0; i < maps_x_.size(); i++) {
    double map_x = maps_x_[i];
    double map_y = maps_y_[i];
    double dist = distance_2d(x, y, map_x, map_y);
    if(dist < closest_len) {
      closest_len = dist;
      closest_wp = i;
    }
  }

  return closest_wp;
}


int Road::next_waypoint(double x, double y, double theta) const {
  int closest_wp = Road::closest_waypoint(x, y);
//  cout << "closest_wp = " << closest_wp << endl;

  double map_x = maps_x_[closest_wp];
  double map_y = maps_y_[closest_wp];

//  cout << "map_x, map_y = " << map_x << ", " << map_y << endl;
  double heading = safe_atan2( ( map_y - y ), ( map_x - x ) );

  double angle = fabs(theta - heading); // was abs

  if (angle > pi()/4) {
    closest_wp++;
    if (closest_wp == maps_x_.size()) {
      closest_wp = 0;
    }
  }

  return closest_wp;
}


vector<double> Road::get_frenet(double x, double y, double theta) const {
  int next_wp = Road::next_waypoint(x,y, theta);

  int prev_wp;
  prev_wp = next_wp - 1;
  if(next_wp == 0) {
    prev_wp  = maps_x_.size() - 1;
  }

  double n_x = maps_x_[next_wp] - maps_x_[prev_wp];
  double n_y = maps_y_[next_wp] - maps_y_[prev_wp];
  double x_x = x - maps_x_[prev_wp];
  double x_y = y - maps_y_[prev_wp];

  // find the projection of x onto n
  double proj_norm = ( x_x * n_x + x_y * n_y ) / ( n_x * n_x + n_y * n_y );
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance_2d(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x_[prev_wp];
  double center_y = 2000 - maps_y_[prev_wp];
  double centerToPos = distance_2d(center_x, center_y, x_x, x_y);
  double centerToRef = distance_2d(center_x, center_y, proj_x, proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += distance_2d(maps_x_[i], maps_y_[i], maps_x_[i+1], maps_y_[i+1]);
  }

  frenet_s += distance_2d(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}


//vector<double> Road::get_xy(double s, double d) const {
//  int prev_wp = -1;
//
//  while(s > maps_s_[prev_wp + 1] && (prev_wp < (int)(maps_s_.size() - 1) )) {
//    prev_wp++;
//  }
//
//  int wp2 = (prev_wp + 1) % maps_x_.size();
//
//  double heading = safe_atan2( ( maps_y_[wp2] - maps_y_[prev_wp] ), ( maps_x_[wp2] - maps_x_[prev_wp] ) );
//
//  // the x,y,s along the segment
//  double seg_s = s - maps_s_[prev_wp];
//  double seg_x = maps_x_[prev_wp] + seg_s * cos(heading);
//  double seg_y = maps_y_[prev_wp] + seg_s * sin(heading);
//
//  double perp_heading = heading - pi()/2;
//
//  double x = seg_x + d * cos(perp_heading);
//  double y = seg_y + d * sin(perp_heading);
//
//  return {x, y};
//}


vector<double> Road::get_xy(double s, double d) const {
  double x0 = s_x_spline_(s);
  double y0 = s_y_spline_(s);
  double dx = s_dx_spline_(s);
  double dy = s_dy_spline_(s);
  double x = x0 + dx * d;
  double y = y0 + dy * d;

  return {x, y};
}


vector<double> Road::get_frenet_velocity(double x, double y, double vx, double vy) const {
  double theta_veh = safe_atan2(vy, vx);
//  cout << "theta_veh = " << theta_veh << endl;
  int next_wp = Road::next_waypoint(x,y, theta_veh);
//  cout << "next_wp = " << next_wp << endl;

  int prev_wp;
  prev_wp = next_wp - 1;
  if(next_wp == 0) {
    prev_wp  = maps_x_.size() - 1;
  }

  double dx = maps_x_[next_wp] - maps_x_[prev_wp];
  double dy = maps_y_[next_wp] - maps_y_[prev_wp];

  double theta_road = safe_atan2(dy, dx);

  double dtheta = theta_veh - theta_road;
  // TODO: double check - might be wrong
//  double s_dot = -vy * sin(dtheta) + vx * cos(dtheta);
//  double d_dot = vy * cos(dtheta) + vx * sin(dtheta);

  double v = sqrt(pow(vx, 2) + pow(vy, 2));
  double d_dot = v * sin(dtheta);
  double s_dot = sqrt(pow(v, 2) - pow(d_dot, 2));


  cout << "vx, vy, dtheta, s_dot, d_dot = " << vx << ", " << vy << ", " << dtheta << ", " << s_dot << ", " << d_dot << endl;

  return {s_dot, d_dot};
}


int Road::get_lane(double d) const {
  return floor(d / LANE_WIDTH);
}


double Road::s_diff(double s1, double s2) const {
  double diff;
  if (s1 > s2) {
    diff = s1 - s2;
  } else if (s2 > s1) {
    diff = (max_s_ - s2) + s1;
  } else {
    diff = 0;
  }

  if (diff > (max_s_ / 2)) {
    diff -= max_s_;
  }

  return diff;
}


double Road::s_norm(double s_in) const {
  if (s_in < 0) {
    s_in += max_s_;
  }
  return fmod(s_in, max_s_);
}
