#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>


class Vehicle {
 public:
  // global state variables
  double x_;
  double y_;
  double yaw_; // radians

  // Frenet state variables
  double s_;
  double s_dot_;
  double s_ddot_;
  double d_;
  double d_dot_;
  double d_ddot_;

  double speed_limit_;

  /**
  * Constructor
  * Initialize with telemetry {x, y, yaw, s, d, s_dot, d_dot}.
  */
  Vehicle(std::vector<double> telemetry_comp);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  /**
  * Get current Frenet state vector:
  * [s, s_dot, s_ddot, d, d_dot, d_ddot].
  */
  std::vector<double> state() const;

  /**
  * Get Frenet state vector at time t (in seconds):
  * [s, s_dot, s_ddot, d, d_dot, d_ddot].
  */
  std::vector<double> state_at(double t) const;

  /**
  * Update with telemetry {x, y, yaw, s, d, s_dot, d_dot}.
  * Assumes constant acceleration.
  */
  void update(std::vector<double> telemetry_comp, double dt);

  void update_with_accel(std::vector<double> telemetry_comp, double s_ddot,
                         double d_ddot);

  void print();
};


#endif
