#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>

#include "Eigen-3.3/Eigen/Core"


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

  // filtered s_ddot and d_dot
  double min_s_ddot_;
  double mean_d_dot_;

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
  * Get Frenet state vector at time t (in seconds) assuming min_s_ddot:
  * [s, s_dot, s_ddot, d, d_dot, d_ddot].
  */
  std::vector<double> min_s_ddot_state_at(double t) const;


  /**
  * Update with telemetry {x, y, yaw, s, d, s_dot, d_dot}.
  * Assumes constant acceleration.
  */
  void update(std::vector<double> telemetry_comp, double dt);
  void update_with_accel(std::vector<double> telemetry_comp, double s_ddot,
                         double d_ddot);

  void print();

 private:
  /**
  * Update history vectors.
  */
  void update_history(double s_dot, double d_dot, double s_ddot,
                      double d_ddot, double dt, bool reset);

  // history of values for filtering
  Eigen::VectorXd t_history_;
  Eigen::VectorXd s_dot_history_;
  Eigen::VectorXd s_ddot_history_;
  Eigen::VectorXd d_dot_history_;
};


#endif
