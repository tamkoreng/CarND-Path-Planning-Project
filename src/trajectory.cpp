#include "trajectory.h"

#include <math.h>
#include <algorithm>
#include <iostream>

#include "Eigen-3.3/Eigen/Dense"

#include "helpers.h"
#include "constants.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


//Trajectory::Trajectory() : start_state_(), end_state_(), T_() {}


Trajectory::Trajectory(state start_state, state end_state, double T) {
  start_state_ = start_state;
  end_state_ = end_state;
  T_ = T;
  s_coef_.setZero(6);
  d_coef_.setZero(6);
  s_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);
  s_dot_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);
  s_ddot_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);
  s_jerk_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);
  d_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);
  d_dot_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);
  d_ddot_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);
  d_jerk_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);
  speed_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);
  accel_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);
  jerk_.setZero(TRAJECTORY_N_EVAL_TIME_STEPS);

  Trajectory::eval();
}


Trajectory::~Trajectory() {}


double Trajectory::calculate_cost(MatrixXd S_tgt, MatrixXd D_tgt, state goal, double T_goal) {
  // find index of last time (times_ contains times that may exceed T)
  int idx = TRAJECTORY_N_EVAL_TIME_STEPS - 1;
  while (TRAJECTORY_TIMES(idx, 1) > T_) {
    idx--;
  }
  T_idx_ = idx;

  VectorXd speed_sq = s_dot_.array().pow(2) + d_dot_.array().pow(2);
  VectorXd accel_sq = s_ddot_.array().pow(2) + d_ddot_.array().pow(2);
  VectorXd jerk_sq = s_jerk_.array().pow(2) + d_jerk_.array().pow(2);
  speed_ = speed_sq.array().sqrt();
  accel_ = accel_sq.array().sqrt();
  jerk_ = jerk_sq.array().sqrt();

//  cout << "speed_ = ";
//  for (int i = 0; i <= (T_idx_ + 1); i++) {
//    cout << speed_(i) << ", ";
//  }
//  cout << endl;

  max_speed_ = speed_.head(T_idx_).maxCoeff();
  max_accel_ = accel_.head(T_idx_).maxCoeff();
  max_jerk_ = jerk_.head(T_idx_).maxCoeff();
  min_s_dot_ = s_dot_.head(T_idx_).minCoeff();

  vector<double> nearest_distances = Trajectory::nearest_approach_to_any_vehicle(s_, d_, S_tgt, D_tgt, T_idx_);
  nearest_distance_ = nearest_distances[0];
  nearest_s_ = nearest_distances[1];
  nearest_d_ = nearest_distances[2];

//  collision_cost_ = collision_cost(nearest_distance_);
  collision_cost_ = collision_cost(nearest_s_, nearest_d_);
  s_buffer_cost_ = s_buffer_cost(nearest_s_);
  d_buffer_cost_ = d_buffer_cost(nearest_d_);
  time_diff_cost_ = time_diff_cost(T_, T_goal);
  s_diff_cost_ = s_diff_cost(s_coef_, T_, goal);
  d_diff_cost_ = d_diff_cost(d_coef_, T_, goal);
  max_accel_cost_ = max_accel_cost(accel_, T_idx_);
  total_accel_cost_ = total_accel_cost(accel_, T_idx_, TRAJECTORY_DT);
  max_jerk_cost_ = max_jerk_cost(jerk_, T_idx_);
  total_jerk_cost_ = total_jerk_cost(jerk_, T_idx_, TRAJECTORY_DT);
  efficiency_cost_ = efficiency_cost(speed_, T_idx_);
  stays_on_road_cost_ = stays_on_road_cost(d_, T_idx_);
  exceeds_speed_limit_cost_ = exceeds_speed_limit_cost(speed_, T_idx_);
  lateral_offset_cost_ = lateral_offset_cost(d_, T_idx_);
  backwards_cost_ = backwards_cost();

  double cost = ( TRAJECTORY_COLLISION_WEIGHT           * collision_cost_           ) +
                ( TRAJECTORY_BUFFER_WEIGHT              * s_buffer_cost_            ) +
                ( TRAJECTORY_BUFFER_WEIGHT              * d_buffer_cost_            ) +
                ( TRAJECTORY_TIME_DIFF_WEIGHT           * time_diff_cost_           ) +
                ( TRAJECTORY_S_DIFF_WEIGHT              * s_diff_cost_              ) +
                ( TRAJECTORY_D_DIFF_WEIGHT              * d_diff_cost_              ) +
                ( TRAJECTORY_MAX_ACCEL_WEIGHT           * max_accel_cost_           ) +
                ( TRAJECTORY_TOTAL_ACCEL_WEIGHT         * total_accel_cost_         ) +
                ( TRAJECTORY_MAX_JERK_WEIGHT            * max_jerk_cost_            ) +
                ( TRAJECTORY_TOTAL_JERK_WEIGHT          * total_jerk_cost_          ) +
                ( TRAJECTORY_EFFICIENCY_WEIGHT          * efficiency_cost_          ) +
                ( TRAJECTORY_STAYS_ON_ROAD_WEIGHT       * stays_on_road_cost_       ) +
                ( TRAJECTORY_EXCEEDS_SPEED_LIMIT_WEIGHT * exceeds_speed_limit_cost_ ) +
                ( TRAJECTORY_LATERAL_OFFSET_COST        * lateral_offset_cost_      ) +
                ( 1e9                                   * backwards_cost_           );

  weighted_cost_ = cost;

//  cout << "speed_.size(), T_idx_, max_speed_, s_min, d_min, cost = " << speed_.size() << ", " << T_idx_ << ", " << max_speed_ << ", "
//       << nearest_s_ << ", " << nearest_d_ << ", " << cost << endl;

  return cost;
}


void Trajectory::eval() {
  vector<double> s_start = {start_state_.s, start_state_.s_dot, start_state_.s_ddot};
  vector<double> d_start = {start_state_.d, start_state_.d_dot, start_state_.d_ddot};
  vector<double> s_end = {end_state_.s, end_state_.s_dot, end_state_.s_ddot};
  vector<double> d_end = {end_state_.d, end_state_.d_dot, end_state_.d_ddot};

  s_coef_ = JMT(s_start, s_end, T_);
  d_coef_ = JMT(d_start, d_end, T_);

  // evaluate trajectory polynomials at all time steps
  VectorXd s_dot_coef = polyder(s_coef_);
  VectorXd s_ddot_coef = polyder(s_dot_coef);
  VectorXd s_jerk_coef = polyder(s_ddot_coef);
  VectorXd d_dot_coef = polyder(d_coef_);
  VectorXd d_ddot_coef = polyder(d_dot_coef);
  VectorXd d_jerk_coef = polyder(d_ddot_coef);

  s_ = TRAJECTORY_TIMES * s_coef_;
  s_dot_ = TRAJECTORY_TIMES.leftCols(5) * s_dot_coef;
  s_ddot_ = TRAJECTORY_TIMES.leftCols(4) * s_ddot_coef;
  s_jerk_ = TRAJECTORY_TIMES.leftCols(3) * s_jerk_coef;
  d_ = TRAJECTORY_TIMES * d_coef_;
  d_dot_ = TRAJECTORY_TIMES.leftCols(5) * d_dot_coef;
  d_ddot_ = TRAJECTORY_TIMES.leftCols(4) * d_ddot_coef;
  d_jerk_ = TRAJECTORY_TIMES.leftCols(3) * d_jerk_coef;

  // ensure s < max_s
  auto loop_back = s_.array() > COMMON_MAX_S;
  auto s_filtered = loop_back.select(s_.array() - COMMON_MAX_S, s_);
  s_ = s_filtered;

//  std::cout << "s_ = " << std::endl;
//  std::cout << s_ << std::endl;
//  std::cout << "d_ = " << std::endl;
//  std::cout << d_ << std::endl;
}


VectorXd Trajectory::JMT(vector<double> start, vector<double> end, double T) const{
  double a_0 = start[0];
  double a_1 = start[1];
  double a_2 = start[2] / 2;

  MatrixXd A = MatrixXd(3, 3);
  VectorXd b = VectorXd(3);
  VectorXd a_345 = VectorXd(3);

  A <<      pow(T, 3),      pow(T, 4),       pow(T, 5),
        3 * pow(T, 2),  4 * pow(T, 3),   5 * pow(T, 4),
        6 * T        , 12 * pow(T, 2),  20 * pow(T, 3);

  b <<  end[0] - ( a_0 + a_1 * T + a_2 * pow(T, 2) ),
        end[1] - ( a_1 + 2 * a_2 * T ),
        end[2] - ( 2 * a_2 );

  a_345 = A.inverse() * b;

  VectorXd coef(6);
  coef << a_0, a_1, a_2, a_345[0], a_345[1], a_345[2];

  return coef;
}


vector<double> Trajectory::nearest_approach_to_any_vehicle(VectorXd s_host, VectorXd d_host,
                                                           MatrixXd S_tgt, MatrixXd D_tgt, int T_idx) const {
  // FIXME - must exclude trailing targets

//  int T_idx_half = floor(T_idx / 2);
  int T_idx_half = T_idx;
  MatrixXd delta_S = S_tgt.topRows(T_idx_half + 1) - s_host.head(T_idx_half + 1) * MatrixXd::Ones(1, S_tgt.cols());
  auto loop_back = delta_S.array() < (-COMMON_MAX_S / 2);
  MatrixXd delta_S_loop_back = loop_back.select( delta_S.array() + COMMON_MAX_S, delta_S );
  auto trailing = delta_S_loop_back.array() < 0;
  MatrixXd big = COMMON_INF_DIST * MatrixXd::Ones(delta_S.rows(), delta_S.cols());
  MatrixXd delta_S_leading = trailing.select(big, delta_S_loop_back);

  MatrixXd delta_D = D_tgt.topRows(T_idx_half + 1) - d_host.head(T_idx_half + 1) * MatrixXd::Ones(1, S_tgt.cols());
  MatrixXd dist_sq = delta_S_leading.array().pow(2) + delta_D.array().pow(2);
//  MatrixXd dist_sq = delta_S.array().pow(2) + delta_D.array().pow(2);
  double min_dist_sq = dist_sq.minCoeff();
  double nearest_dist = sqrt(min_dist_sq);

  // nearest longitudinal distance (in host path, front or rear)
  auto in_path = delta_D.array().abs() < COMMON_IN_PATH_D_DIST;
  double nearest_longitudinal = in_path.select(delta_S_leading.array().abs(), big).minCoeff();
//  double nearest_longitudinal = in_path.select(delta_S.array().abs(), big).minCoeff();

  // nearest lateral distance (near host s position)
  auto adjacent = delta_S_leading.array().abs() < COMMON_ADJACENT_S_DIST;
//  auto adjacent = delta_S.array().abs() < COMMON_ADJACENT_S_DIST;
  double nearest_lateral = adjacent.select(delta_D.array().abs(), big).minCoeff();

  return {nearest_dist, nearest_longitudinal, nearest_lateral};
}


//double Trajectory::collision_cost(double nearest_dist) const {
//  double cost = 0;
//  if (nearest_dist < (2 * TRAJECTORY_VEHICLE_RADIUS)) {
//    cost = 1;
//  }
//  return cost;
//}
double Trajectory::collision_cost(double nearest_longitudinal, double nearest_lateral) const {
  double cost = 0;
  if ( (nearest_longitudinal < 4.5 ) | (nearest_lateral < 2.5) ) {
    cost = 1;
  }
  return cost;
}


double Trajectory::s_buffer_cost(double nearest_longitudinal) const {
  double cost = 0;
  double s_gap = max_speed_ * FOLLOWER_T_GAP + FOLLOWER_R0;
//  if (nearest_longitudinal < TRAJECTORY_S_BUFFER) {
//    cost = logistic( TRAJECTORY_S_BUFFER / nearest_longitudinal );
//  }
  if (nearest_longitudinal < s_gap) {
    cost = logistic( s_gap / nearest_longitudinal );
  }
  return cost;
}


double Trajectory::d_buffer_cost(double nearest_lateral) const {
  double cost = 0;
  if (nearest_lateral < TRAJECTORY_D_BUFFER) {
    cost = logistic( TRAJECTORY_D_BUFFER / nearest_lateral );
  }
  return cost;
}


double Trajectory::time_diff_cost(double T, double T_goal) const {
  return logistic(fabs(T - T_goal) / T_goal);
}


double Trajectory::s_diff_cost(VectorXd s_coef, double T, state goal) const {
  // could alternatively interpolate between eval_traj points at T
  VectorXd s_dot_coef = polyder(s_coef);
  VectorXd s_ddot_coef = polyder(s_dot_coef);
  vector<double> actual(3);
  actual[0] = polyeval(s_coef, T);
  actual[1] = polyeval(s_dot_coef, T);
  actual[2] = polyeval(s_ddot_coef, T);
  vector<double> goal_s = {goal.s, goal.s_dot, goal.s_ddot};

  double cost = 0;
  for (int i = 0; i < 3; i++) {
    double diff = fabs(actual[i] - goal_s[i]);
    cost += logistic(diff / PTG_SIGMA_S[i]);
  }

  return cost;
}


double Trajectory::d_diff_cost(VectorXd d_coef, double T, state goal) const {
  // could alternatively interpolate between eval_traj points at T
  VectorXd d_dot_coef = polyder(d_coef);
  VectorXd d_ddot_coef = polyder(d_dot_coef);
  vector<double> actual(3);
  actual[0] = polyeval(d_coef, T);
  actual[1] = polyeval(d_dot_coef, T);
  actual[2] = polyeval(d_ddot_coef, T);
  vector<double> goal_d = {goal.d, goal.d_dot, goal.d_ddot};

  double cost = 0;
  for (int i = 0; i < 3; i++) {
    if (PTG_SIGMA_D[i] > 0) {
      double diff = fabs(actual[i] - goal_d[i]);
      cost += logistic(diff / PTG_SIGMA_D[i]);
    }
  }

  return cost;
}


double Trajectory::max_accel_cost(VectorXd accel, int T_idx) const {
  double max_accel = accel.head(T_idx).maxCoeff();

  double cost = 0;
  if (max_accel > COMMON_MAX_ACCEL) {
    cost = 1;
  }
  return cost;
}


double Trajectory::total_accel_cost(VectorXd accel, int T_idx, double dt) const {
  double total_accel = accel.head(T_idx).sum() * dt;
  double accel_per_sec = total_accel / ( (T_idx) * dt );
  return logistic( accel_per_sec / TRAJECTORY_EXPECTED_ACCEL_IN_ONE_SEC );
}


double Trajectory::max_jerk_cost(VectorXd jerk, int T_idx) const {
  double max_jerk = jerk.head(T_idx + 1).maxCoeff();

  double cost = 0;
  if (max_jerk > COMMON_MAX_JERK) {
    cost = 1;
  }
  return cost;
}


double Trajectory::total_jerk_cost(VectorXd jerk, int T_idx, double dt) const {
  double total_jerk = jerk.head(T_idx + 1).sum() * dt;
  double jerk_per_sec = total_jerk / ( T_idx * dt );
  return logistic( jerk_per_sec / TRAJECTORY_EXPECTED_JERK_IN_ONE_SEC );
}


double Trajectory::efficiency_cost(VectorXd speed, int T_idx) const {
  double avg_v = speed.head(T_idx + 1).mean();
  double tgt_avg_v = COMMON_SPEED_LIMIT;

  double cost = 0;
  if (avg_v < tgt_avg_v) {
      cost = logistic( ( tgt_avg_v - avg_v ) / avg_v );
  }
  return cost;
}


double Trajectory::stays_on_road_cost(VectorXd d, int T_idx) const {
  double d_min = d.head(T_idx + 1).minCoeff();
  double d_max = d.head(T_idx + 1).maxCoeff();

  double cost = 0;
  if ( ( d_min < TRAJECTORY_LEFT_LIMIT ) | ( d_max > TRAJECTORY_RIGHT_LIMIT ) ) {
    cost = 1;
  }
  return cost;
}


double Trajectory::exceeds_speed_limit_cost(VectorXd speed, int T_idx) const {
  double max_speed = speed.head(T_idx).maxCoeff();

  double cost = 0;
  if (max_speed > (COMMON_SPEED_LIMIT + 1.5)) { // WAS COMMON_SPEED_LIMIT + 2, then 1.5
    cost = 1;
  }
  return cost;
}


double Trajectory::lateral_offset_cost(VectorXd d, int T_idx) const {
  double w = COMMON_LANE_WIDTH;
  VectorXd d_over_w = d.head(T_idx).array() / w;
  VectorXd offset = d.head(T_idx).array() - ( w * d_over_w.array().cast<int>().cast<double>() ) - (w / 2);
  double mean_abs_offset = offset.array().abs().mean();
  double cost = logistic( mean_abs_offset / 0.1 );
  return cost;
}


double Trajectory::backwards_cost() const {
  double cost = 0;
  if (min_s_dot_ < 0) {
    cost = 1;
  }
  return cost;
}
