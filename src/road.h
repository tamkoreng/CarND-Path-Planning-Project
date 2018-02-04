#ifndef ROAD_H
#define ROAD_H

#include <vector>

#include "spline.h"


// Refactored waypoint and coordinate transformation functions from starter code
// main.cpp


class Road {
public:
  double speed_limit_; // m/s
  double lane_width_; // m
  int n_lanes_;
  double max_s_; // m
  std::vector<double> maps_x_;
  std::vector<double> maps_y_;
  std::vector<double> maps_s_;
  std::vector<double> maps_dx_;
  std::vector<double> maps_dy_;

  // splines for get_xy
  tk::spline s_x_spline_;
  tk::spline s_y_spline_;
  tk::spline s_dx_spline_;
  tk::spline s_dy_spline_;

  /**
  * Constructor
  * Initialize with map waypoints.
  */
  Road(double speed_limit, double lane_width, int n_lanes, double max_s,
       std::vector<double> maps_x, std::vector<double> maps_y,
       std::vector<double> maps_s, std::vector<double> maps_dx,
       std::vector<double> maps_dy);

  /**
  * Destructor
  */
  virtual ~Road();

  int closest_waypoint(double x, double y) const;

  int next_waypoint(double x, double y, double theta) const;

  /**
  * Transform from Cartesian x,y coordinates to Frenet s,d coordinates.
  */
  std::vector<double> get_frenet(double x, double y, double theta) const;

  /**
  * Transform from Frenet s,d coordinates to Cartesian x,y.
  */
  std::vector<double> get_xy(double s, double d) const;

  /**
  * Transform Cartesian vx, vy to Frenet s_dot, d_dot.
  */
  std::vector<double> get_frenet_velocity(double x, double y, double vx,
                                          double vy) const;

  /**
  * Get lane index from d coordinate.
  */
  int get_lane(double d) const;

  /**
  * Find difference between coordinates - taking account of loop back.
  */
  double s_diff(double s1, double s2) const;

  /**
  * Compensate s coordinate for loop back.
  */
  double s_norm(double s_in) const;

  /**
  * Find speed in Frenet frame.
  */
  double s_dot(double v, double s, double d) const;
};


#endif
