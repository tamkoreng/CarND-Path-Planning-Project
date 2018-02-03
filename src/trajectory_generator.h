#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <map>
#include <random>

#include "Eigen-3.3/Eigen/Core"

#include "road.h"
#include "vehicle.h"
#include "predictor.h"
#include "trajectory.h"


struct trajectory {
  Eigen::VectorXd s_coef; // 5th degree polynomial coefficients
  Eigen::VectorXd d_coef; // 5th degree polynomial coefficients
  double T; // trajectory duration (in s)
};


class TrajectoryGenerator {
 public:
  /**
  * Constructor
  */
  TrajectoryGenerator(const Road &road, Predictor &predictor);


  /**
  * Destructor
  */
  virtual ~TrajectoryGenerator();


  /**
  * Find the best trajectory according weighted cost functions.
  *
  * INPUTS
  * start_state - {s, s_dot, s_ddot, d, d_dot, d_ddot}
  * end_state - {s, s_dot, s_ddot, d, d_dot, d_ddot}
  * T - the desired time in seconds at which we will be at the goal (relative to now as t=0)
  *
  * OUTPUT
  * {best_s, best_d, best_t} where best_s are the 6 coefficients representing s(t)
  * best_d gives coefficients for d(t) and best_t gives duration associated w/
  * this trajectory.
  */
  trajectory PTG(std::vector<double> start_state, std::vector<double> end_state, double T);
//  Trajectory PTG(std::vector<double> start_state, std::vector<double> end_state, double T);


  /**
  * Generate x/y points for simulator.
  * Must be called at each time step.
  */
  std::vector< std::vector<double> > get_points(trajectory traj, int prev_size);
//  std::vector< std::vector<double> > get_points(Trajectory traj, int prev_size);

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
//  std::vector< std::vector<double> > keep_lane(int lane, double t_horizon, int prev_size);
  std::vector< std::vector<double> > keep_lane(double t_horizon, int prev_size);


  /**
  * Find state (s, s_dot, s_ddot) at t_horizon following jerk-limited
  * trapezoidal acceleration trajectory.
  */
  std::vector<double> trapezoidal_accel(double t_horizon);
  std::vector<double> trapezoidal_accel(double t_horizon, double v_goal, double a_max, double jerk_max);


  /**
  * Get next_x_vals, next_y_vals for given goal.
  */
  std::vector< std::vector<double> > get_points_for_goal(std::vector<double> goal_state,
                                                         double t_horizon,
                                                         int prev_size);

  std::vector<double> prev_pts_s_;
  std::vector<double> prev_pts_s_dot_;
  std::vector<double> prev_pts_s_ddot_;
  std::vector<double> prev_pts_d_;
  std::vector<double> prev_pts_d_dot_;
  std::vector<double> prev_pts_d_ddot_;
  std::vector<double> prev_pts_x_;
  std::vector<double> prev_pts_y_;


 private:
  const Road &road_;
  Predictor &predictor_;

//  std::vector<double> prev_pts_s_;
//  std::vector<double> prev_pts_s_dot_;
//  std::vector<double> prev_pts_s_ddot_;
//  std::vector<double> prev_pts_d_;
//  std::vector<double> prev_pts_d_dot_;
//  std::vector<double> prev_pts_d_ddot_;
//  std::vector<double> prev_pts_x_;
//  std::vector<double> prev_pts_y_;

//  Trajectory prev_traj_;

  std::mt19937 gen_;

  /**
  * Return perturbed version of goal end state.
  */
  std::vector<double> perturb_goal(std::vector<double> end_state, double sigma_multiplier);

  // last element is T
  std::vector<double> perturb_goal(std::vector<double> start_state, std::vector<double> end_state, double T, double sigma_multiplier);

  double perturb_t(double T);
};

#endif
