#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <map>
#include <random>

#include "Eigen-3.3/Eigen/Core"

#include "road.h"
#include "vehicle.h"
#include "predictor.h"


struct trajectory {
  Eigen::VectorXd s_coef;
  Eigen::VectorXd d_coef;
  double T;
};


class TrajectoryGenerator {
 public:
  /**
  * Constructor
  */
  TrajectoryGenerator(const Road &road, const Predictor &predictor);


  /**
  * Destructor
  */
  virtual ~TrajectoryGenerator();


//  /**
//  * Find the best trajectory according weighted cost functions.
//  *
//  * INPUTS
//  * start_state - {s, s_dot, s_ddot, d, d_dot, d_ddot}
//  * end_state - {s, s_dot, s_ddot, d, d_dot, d_ddot}
//  * T - the desired time in seconds at which we will be at the goal (relative to now as t=0)
//  *
//  * OUTPUT
//  * {best_s, best_d, best_t} where best_s are the 6 coefficients representing s(t)
//  * best_d gives coefficients for d(t) and best_t gives duration associated w/
//  * this trajectory.
//  */
//  trajectory PTG(std::vector<double> start_state, std::vector<double> end_state, double T);


  /**
  * Generate x/y points for simulator.
  * Must be called at each time step.
  */
  std::vector< std::vector<double> > get_points(trajectory traj, int prev_size);

  /**
  * Get host state {s, s_dot, s_ddot, d, d_dot, d_ddot} based on previous path size.
  */
  std::vector<double> get_host_state(int prev_size);


  /**
  * Get host x/y based on previous path size.
  */
  std::vector<double> get_host_xy(int prev_size);


  /**
  * Find next_x_vals, next_y_vals to keep host in lane.
  */
  std::vector< std::vector<double> > keep_lane(double t_horizon, int prev_size);


  /**
  * Find state (s, s_dot, s_ddot) at t_horizon following jerk-limited
  * trapezoidal acceleration trajectory.
  */
  std::vector<double> trapezoidal_accel(double t_horizon);


  /**
  * Call appropriate method to generate trajectory for given state.
  * Uses trajectory to generate waypoints.
  */
  void realize_state(string state, int prev_size);

  void change_lane_left();

  void change_lane_right();

  void prep_lane_change_left(int target_id);

  void prep_lane_change_right(int target_id);

  void keep_lane();

 private:
  const Road &road_;
  const Predictor &predictor_;

  std::vector<double> prev_pts_s_;
  std::vector<double> prev_pts_s_dot_;
  std::vector<double> prev_pts_s_ddot_;
  std::vector<double> prev_pts_d_;
  std::vector<double> prev_pts_d_dot_;
  std::vector<double> prev_pts_d_ddot_;
  std::vector<double> prev_pts_x_;
  std::vector<double> prev_pts_y_;

  std::mt19937 gen_;

  string state_;
  trajectory current_traj_;

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


//  /**
//  * Return perturbed version of goal end state.
//  */
//  std::vector<double> perturb_goal(std::vector<double> end_state);
//
//
//  double calculate_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  double nearest_approach(trajectory traj, const Vehicle &veh) const;
//
//
//  /**
//  Calculate the closest distance to any target during a trajectory.
//  */
//  double nearest_approach_to_any_vehicle(trajectory traj) const;
//
//
//  /**
//  * Penalizes trajectories that span a duration which is longer or shorter
//  * than the duration requested.
//  */
//  double time_diff_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  * Penalizes trajectories whose s coordinate (and derivatives) differ from the
//  * goal.
//  */
//  double s_diff_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  * Penalizes trajectories whose d coordinate (and derivatives) differ from
//  * the goal.
//  */
//  double d_diff_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  * Binary cost function which penalizes collisions.
//  */
//  double collision_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  * Penalizes getting close to other vehicles.
//  */
//  double buffer_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  * TODO: implement
//  */
//  double stays_on_road_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  * TODO: implement
//  */
//  double exceeds_speed_limit_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  * Rewards high average speeds.
//  */
//  double efficiency_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  * Binary cost function to penalize high peak acceleration (in s direction).
//  */
//  double max_accel_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  * Penalize high integrated acceleration (in s direction).
//  */
//  double total_accel_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  * Penalize high peak jerk (in s direction).
//  */
//  double max_jerk_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//
//
//  /**
//  Penalize high integrated jerk (in s direction).
//  */
//  double total_jerk_cost(trajectory traj, std::vector<double> start_state, std::vector<double> end_state, double T) const;
//};

#endif
