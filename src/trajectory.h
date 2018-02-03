#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <map>
#include <random>

#include "Eigen-3.3/Eigen/Core"


struct state {
//  state(double _s = 0, double _s_dot = 0, double _s_ddot = 0,
//        double _d = 0, double _d_dot = 0, double _d_ddot = 0) :
//          s(_s), s_dot(_s_dot), s_ddot(_s_ddot),
//          d(_d), d_dot(_d_dot), d_ddot(_d_ddot) {}
//  double s = 0;
//  double s_dot = 0;
//  double s_ddot = 0;
//  double d = 0;
//  double d_dot = 0;
//  double d_ddot = 0;
  double s;
  double s_dot;
  double s_ddot;
  double d;
  double d_dot;
  double d_ddot;
};


class Trajectory {
 public:
  state start_state_;
  state end_state_;
  double T_;

  double dt_;
  int T_idx_;

  Eigen::VectorXd s_coef_;
  Eigen::VectorXd d_coef_;

  Eigen::VectorXd s_;
  Eigen::VectorXd s_dot_;
  Eigen::VectorXd s_ddot_;
  Eigen::VectorXd s_jerk_;
  Eigen::VectorXd d_;
  Eigen::VectorXd d_dot_;
  Eigen::VectorXd d_ddot_;
  Eigen::VectorXd d_jerk_;

  Eigen::VectorXd speed_;
  Eigen::VectorXd accel_;
  Eigen::VectorXd jerk_;

  double max_speed_;
  double max_accel_;
  double max_jerk_;
  double min_s_dot_;

  double nearest_distance_;
  double nearest_s_;
  double nearest_d_;

  double collision_cost_;
  double s_buffer_cost_;
  double d_buffer_cost_;
  double time_diff_cost_;
  double s_diff_cost_;
  double d_diff_cost_;
  double max_accel_cost_;
  double total_accel_cost_;
  double max_jerk_cost_;
  double total_jerk_cost_;
  double efficiency_cost_;
  double stays_on_road_cost_;
  double exceeds_speed_limit_cost_;
  double lateral_offset_cost_;
  double backwards_cost_;

  double weighted_cost_;

//  Trajectory();
  Trajectory(state start_state, state end_state, double T);

  virtual ~Trajectory();

  double calculate_cost(Eigen::MatrixXd S_tgt, Eigen::MatrixXd D_tgt, state goal, double T_goal);

 private:

  void eval();

  /**
  * Calculate the Jerk Minimizing Trajectory that connects the initial state
  * to the final state in time T.
  *
  * INPUTS
  * start - the vehicles start location given as a length three array
  *     corresponding to initial values of [s, s_dot, s_double_dot]
  *
  * end   - the desired end state for vehicle. Like "start" this is a
  *     length three array.
  *
  * T     - The duration, in seconds, over which this maneuver should occur.
  *
  * OUTPUT
  * an array of length 6, each value corresponding to a coefficient in the polynomial
  * s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
  *
  * EXAMPLE
  *
  * > JMT( [0, 10, 0], [10, 10, 0], 1)
  * [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */
  Eigen::VectorXd JMT(std::vector<double> start, std::vector<double> end, double T) const;

  /**
  * Calculate the closest distance to any target during a trajectory.
  * Returns {nearest, nearest_longitudinal, nearest_lateral}.
  */
  std::vector<double> nearest_approach_to_any_vehicle(Eigen::VectorXd s_host, Eigen::VectorXd d_host,
                                                      Eigen::MatrixXd S_tgt, Eigen::MatrixXd D_tgt, int T_idx) const;


  /**
  * Binary cost function which penalizes collisions.
  */
//  double collision_cost(double nearest_dist) const;
  double collision_cost(double nearest_longitudinal, double nearest_lateral) const;


  /**
  * Penalizes getting close to other vehicles.
  */
  double s_buffer_cost(double nearest_longitudinal) const;
  double d_buffer_cost(double nearest_lateral) const;


  /**
  * Penalizes trajectories that span a duration which is longer or shorter
  * than the duration requested.
  */
  double time_diff_cost(double T, double T_goal) const;


  /**
  * Penalizes trajectories whose s coordinate (and derivatives) differ from the
  * goal.
  */
  double s_diff_cost(Eigen::VectorXd s_coef, double T, state goal) const;


  /**
  * Penalizes trajectories whose d coordinate (and derivatives) differ from
  * the goal.
  */
  double d_diff_cost(Eigen::VectorXd d_coef, double T, state goal) const;


  /**
  * Binary cost function to penalize high peak acceleration (in s direction).
  */
  double max_accel_cost(Eigen::VectorXd accel, int T_idx) const;


  /**
  * Penalize high integrated acceleration (in s direction).
  */
  double total_accel_cost(Eigen::VectorXd accel, int T_idx, double dt) const;


  /**
  * Penalize high peak jerk (in s direction).
  */
  double max_jerk_cost(Eigen::VectorXd jerk, int T_idx) const;


  /**
  Penalize high integrated jerk (in s direction).
  */
  double total_jerk_cost(Eigen::VectorXd jerk, int T_idx, double dt) const;


  /**
  * Rewards high average speeds.
  */
  double efficiency_cost(Eigen::VectorXd speed, int T_idx) const;


  /**
  * Penalize getting too close to road edge.
  */
  double stays_on_road_cost(Eigen::VectorXd d, int T_idx) const;


  /**
  * Penalize exceeding the speed limit.
  */
  double exceeds_speed_limit_cost(Eigen::VectorXd speed, int T_idx) const;


  /**
  * Penalize driving outside the lane.
  */
  double lateral_offset_cost(Eigen::VectorXd d, int T_idx) const;

  /**
  * Penalize driving backwards.
  */
  double backwards_cost() const;
};

#endif
