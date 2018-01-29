#include "trajectory_generator.h"

#include <math.h>
#include <algorithm>
#include <iostream>

#include "Eigen-3.3/Eigen/Dense"
#include "spline.h"

#include "helpers.h"
#include "constants.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


mt19937 make_seeded_engine() {
  random_device rd;
  return mt19937(rd());
}


TrajectoryGenerator::TrajectoryGenerator(const Road &road, Predictor &predictor) :
  road_(road),
  predictor_(predictor)
  {
  gen_ = make_seeded_engine();
}


TrajectoryGenerator::~TrajectoryGenerator() {}


trajectory TrajectoryGenerator::PTG(vector<double> start_state, vector<double> end_state, double T) {
//Trajectory TrajectoryGenerator::PTG(vector<double> start_state, vector<double> end_state, double T) {
  double max_cost = 2e6;
  int max_iteration = 5;
  double current_iteration = 1;
//  double time_step = PTG_SIGMA_T / PTG_N_TIME_STEPS;

  // calculate predicted target states
  vector<MatrixXd> target_predictions = predictor_.target_predictions(TRAJECTORY_MAX_EVAL_TIME, TRAJECTORY_N_EVAL_TIME_STEPS);
  MatrixXd S_tgt = target_predictions[0];
  MatrixXd D_tgt = target_predictions[1];

  // generate unperturbed trajectory
  state start = {start_state[0], start_state[1], start_state[2], start_state[3], start_state[4], start_state[5]};
  state goal = {end_state[0], end_state[1], end_state[2], end_state[3], end_state[4], end_state[5]};
  Trajectory traj_orig = Trajectory(start, goal, T);
  double current_cost = traj_orig.calculate_cost(S_tgt, D_tgt, goal, T);
  bool will_collide = ( traj_orig.collision_cost_ > 1e-6 );
  double max_speed = traj_orig.max_speed_;
  trajectory traj_out = {traj_orig.s_coef_, traj_orig.d_coef_, T};

  // perturb trajectory if needed
  if (current_cost > max_cost) {
//    cout << "running PTG - current_cost = " << current_cost << endl;
//    cout << "max_v, max_a, max_j, min_d, min_s = " << traj_orig.max_speed_ << ", "
//         << traj_orig.max_accel_ << ", " << traj_orig.max_jerk_ << ", " << traj_orig.nearest_d_ << ", " << traj_orig.nearest_s_ << endl;
//    if (traj_orig.max_speed_ > 30) {
//      cout << "Speed over 30 m/s." << endl;
//    }
  }
  while ( (current_cost > max_cost) & (current_iteration <= max_iteration) ) {
    vector< vector<double> > end_states;
    vector<double> times;

    // include unperturbed state
    end_states.push_back(end_state);
    times.push_back(T);

    // generate end states
//    double t = max(T - PTG_N_TIME_STEPS * time_step, TRAJECTORY_TIME_STEP);
  //  while (t <= (T + PTG_N_TIME_STEPS * time_step)) {
  //    // loop over time steps
  //    for (int i = 0; i < PTG_N_SAMPLES; i++) {
  //      // loop over perturbations
  //      vector<double> perturbed = perturb_goal(end_state);
  //      end_states.push_back(perturbed);
  //      times.push_back(t);
  //    }
  //    t += time_step;
  //  }

    // perturbations in s and d (but not t)
    for (int i = 0; i < PTG_N_SAMPLES; i++) {
      // loop over perturbations
      vector<double> perturbed = perturb_goal(end_state, pow(1.5, current_iteration));
      end_states.push_back(perturbed);
      times.push_back(perturb_t(T));
    }

    // generate jerk minimal trajectories
    vector<Trajectory> trajectories;
    vector<double> costs;

    for (int i = 0; i < end_states.size(); i++) {
      state start = {start_state[0], start_state[1], start_state[2], start_state[3], start_state[4], start_state[5]};
      state end = {end_states[i][0], end_states[i][1], end_states[i][2], end_states[i][3], end_states[i][4], end_states[i][5]};

      Trajectory traj = Trajectory(start, end, times[i]);
      double cost = traj.calculate_cost(S_tgt, D_tgt, goal, T);
      trajectories.push_back(traj);
      costs.push_back(cost);
    }

    // find minimum cost trajectory
    int min_idx = distance( costs.begin(), min_element(costs.begin(), costs.end()) );
    traj_out = {trajectories[min_idx].s_coef_, trajectories[min_idx].d_coef_, trajectories[min_idx].T_};
//    Trajectory traj_out = trajectories[min_idx];
    will_collide = ( trajectories[min_idx].collision_cost_ > 1e-4 );
    current_cost = costs[min_idx];
    max_speed = trajectories[min_idx].max_speed_;

//    cout << "trajectory_generator (it " << current_iteration << "): max_speed [mph], s_min, d_min, cost = " << trajectories[min_idx].max_speed_ * 2.237 << ", "
//         << trajectories[min_idx].nearest_s_ << ", " << trajectories[min_idx].nearest_d_ << ", " << current_cost << endl;

    current_iteration++;
  }
//  prev_traj_ = traj_out;

 // update lane selector
  predictor_.update_lane_selector(traj_out.T, will_collide);
  if (max_speed > (COMMON_SPEED_LIMIT + 0) ) {
    cout << "PTG: max_speed, cost, iterations = " << max_speed * 2.237 << ", " << current_cost
         << ", " << current_iteration - 1 << endl;
  }

  return traj_out;
}


vector< vector<double> > TrajectoryGenerator::get_points(trajectory traj, int prev_size) {
//vector< vector<double> > TrajectoryGenerator::get_points(Trajectory traj, int prev_size) {
  double t_epsilon = 0.001;

  // initialize vectors of new path points
  vector<double> new_pts_s;
  vector<double> new_pts_s_dot;
  vector<double> new_pts_s_ddot;
  vector<double> new_pts_d;
  vector<double> new_pts_d_dot;
  vector<double> new_pts_d_ddot;
  vector<double> new_pts_x;
  vector<double> new_pts_y;

  new_pts_s.resize(TRAJECTORY_N_POINTS);
  new_pts_s_dot.resize(TRAJECTORY_N_POINTS);
  new_pts_s_ddot.resize(TRAJECTORY_N_POINTS);
  new_pts_d.resize(TRAJECTORY_N_POINTS);
  new_pts_d_dot.resize(TRAJECTORY_N_POINTS);
  new_pts_d_ddot.resize(TRAJECTORY_N_POINTS);
  new_pts_x.resize(TRAJECTORY_N_POINTS);
  new_pts_y.resize(TRAJECTORY_N_POINTS);

  double t_max;
  int pts_used = TRAJECTORY_N_POINTS - prev_size;
  int idx = 0;
  vector<double> start_state;

  if (prev_pts_s_.size() > 0) {
//    std::cout << "trajectory_generator: prev_pts_s_.size() = " << prev_pts_s_.size() << std::endl;
//    predictor_.host_.print();
    // previous point vectors already initialized
    int idx_start = pts_used + TRAJECTORY_N_OVERLAP - 1;
    t_max = ( TRAJECTORY_N_POINTS - TRAJECTORY_N_OVERLAP ) * TRAJECTORY_TIME_STEP;

    // reuse some of the previous points
    for (int i = pts_used; i <= idx_start; i++) {
      new_pts_s[idx] = prev_pts_s_[i];
      new_pts_s_dot[idx] = prev_pts_s_dot_[i];
      new_pts_s_ddot[idx] = prev_pts_s_ddot_[i];
      new_pts_d[idx] = prev_pts_d_[i];
      new_pts_d_dot[idx] = prev_pts_d_dot_[i];
      new_pts_d_ddot[idx] = prev_pts_d_ddot_[i];
      new_pts_x[idx] = prev_pts_x_[i];
      new_pts_y[idx] = prev_pts_y_[i];
      idx++;
    }

    // get state at starting index
    start_state = {prev_pts_s_[idx_start],
                   prev_pts_s_dot_[idx_start],
                   prev_pts_s_ddot_[idx_start],
                   prev_pts_d_[idx_start],
                   prev_pts_d_dot_[idx_start],
                   prev_pts_d_ddot_[idx_start]};

//    // use PTG to generate new points from start index
//    traj = TrajectoryGenerator::PTG(start_state, end_state, T);

  } else {
//    std::cout << "trajectory_generator: first time called" << std::endl;
//    predictor_.host_.print();

    // first time called
//    t_max = TRAJECTORY_N_POINTS * TRAJECTORY_TIME_STEP;
    t_max = ( TRAJECTORY_N_POINTS - TRAJECTORY_N_OVERLAP ) * TRAJECTORY_TIME_STEP;
    for (int i = 0; i < TRAJECTORY_N_OVERLAP; i++) {
      new_pts_s[idx] = predictor_.host_.s_;
      new_pts_s_dot[idx] = predictor_.host_.s_dot_;
      new_pts_s_ddot[idx] = predictor_.host_.s_ddot_;
      new_pts_d[idx] = predictor_.host_.d_;
      new_pts_d_dot[idx] = predictor_.host_.d_dot_;
      new_pts_d_ddot[idx] = predictor_.host_.d_ddot_;
      new_pts_x[idx] = predictor_.host_.x_;
      new_pts_y[idx] = predictor_.host_.y_;
      idx++;
    }

    // use host state for start state
//    traj = TrajectoryGenerator::PTG(predictor_.host_.state(), end_state, T);
    start_state = predictor_.host_.state();
  }

  // get polynomials for new trajectory derivatives
  VectorXd s_dot_poly = polyder(traj.s_coef);
//  VectorXd s_dot_poly = polyder(traj.s_coef_);
  VectorXd s_ddot_poly = polyder(s_dot_poly);
  VectorXd d_dot_poly = polyder(traj.d_coef);
//  VectorXd d_dot_poly = polyder(traj.d_coef_);
  VectorXd d_ddot_poly = polyder(d_dot_poly);

  double t = TRAJECTORY_TIME_STEP; // t = 0 @ idx_start
  while (t < (min(t_max, traj.T) + t_epsilon)) {
//  while (t < (min(t_max, traj.T_) + t_epsilon)) {
    // evaluate trajectory polynomials to get points
    new_pts_s[idx] = road_.s_norm(polyeval(traj.s_coef, t));
    new_pts_s_dot[idx] = polyeval(s_dot_poly, t);
    new_pts_s_ddot[idx] = polyeval(s_ddot_poly, t);
    new_pts_d[idx] = polyeval(traj.d_coef, t);
    new_pts_d_dot[idx] = polyeval(d_dot_poly, t);
    new_pts_d_ddot[idx] = polyeval(d_ddot_poly, t);
//    double new_s_dot = polyeval(s_dot_poly, t);
//    if (new_s_dot > 2e-2) {
//      new_pts_s[idx] = road_.s_norm(polyeval(traj.s_coef, t));
//      new_pts_s_dot[idx] = new_s_dot;
//      new_pts_s_ddot[idx] = polyeval(s_ddot_poly, t);
//      new_pts_d[idx] = polyeval(traj.d_coef, t);
//      new_pts_d_dot[idx] = polyeval(d_dot_poly, t);
//      new_pts_d_ddot[idx] = polyeval(d_ddot_poly, t);
//    } else {
//      new_pts_s[idx] = new_pts_s[idx-1];
//      new_pts_s_dot[idx] = new_pts_s_dot[idx-1];
//      new_pts_s_ddot[idx] = new_pts_s_ddot[idx-1];
//      new_pts_d[idx] = new_pts_d[idx-1];
//      new_pts_d_dot[idx] = 0;
//      new_pts_d_ddot[idx] = 0;
//    }

    // convert to x/y coordinates
    vector<double> next_pt = road_.get_xy(new_pts_s[idx], new_pts_d[idx]);
    new_pts_x[idx] = next_pt[0];
    new_pts_y[idx] = next_pt[1];

    t += TRAJECTORY_TIME_STEP;
    idx++;
  }

  if (traj.T < t_max) {
//  if (traj.T_ < t_max) {
    // extend path assuming constant acceleration from last point
    double t0 = t - TRAJECTORY_TIME_STEP;
    double s0 = new_pts_s[idx-1];
    double s_dot0 = new_pts_s_dot[idx-1];
    double s_ddot0 = new_pts_s_ddot[idx-1];
    double d0 = new_pts_d[idx-1];
    double d_dot0 = new_pts_d_dot[idx-1];
    double d_ddot0 = new_pts_d_ddot[idx-1];

    while (t < (t_max + t_epsilon)) {
      new_pts_s[idx] = road_.s_norm(s0 + s_dot0 * (t - t0) + 0.5 * s_ddot0 * pow(t - t0, 2));
      new_pts_s_dot[idx] = (s_dot0 + s_ddot0 * (t - t0));
      new_pts_s_ddot[idx] = (s_ddot0);
      new_pts_d[idx] = (d0 + d_dot0 * (t - t0) + 0.5 * d_ddot0 * pow(t - t0, 2));
      new_pts_d_dot[idx] = (d_dot0 + d_ddot0 * (t - t0));
      new_pts_d_ddot[idx] = (d_ddot0);

      // convert to x/y coordinates
      vector<double> next_pt = road_.get_xy(new_pts_s[idx], new_pts_d[idx]);
      new_pts_x[idx] = (next_pt[0]);
      new_pts_y[idx] = (next_pt[1]);

      t += TRAJECTORY_TIME_STEP;
      idx++;
    }
  }

  // cache new points
  prev_pts_s_ = new_pts_s;
  prev_pts_s_dot_ = new_pts_s_dot;
  prev_pts_s_ddot_ = new_pts_s_ddot;
  prev_pts_d_ = new_pts_d;
  prev_pts_d_dot_ = new_pts_d_dot;
  prev_pts_d_ddot_ = new_pts_d_ddot;
  prev_pts_x_ = new_pts_x;
  prev_pts_y_ = new_pts_y;

  return {new_pts_x, new_pts_y};
}


vector<double> TrajectoryGenerator::get_host_state(int prev_size) {
  int idx = TRAJECTORY_N_POINTS - prev_size - 1;
  return {prev_pts_s_[idx],
          prev_pts_s_dot_[idx],
          prev_pts_s_ddot_[idx],
          prev_pts_d_[idx],
          prev_pts_d_dot_[idx],
          prev_pts_d_ddot_[idx]};
}


vector<double> TrajectoryGenerator::get_host_xy(int prev_size) {
  int idx = TRAJECTORY_N_POINTS - prev_size - 1;
  return {prev_pts_x_[idx], prev_pts_y_[idx]};
}


vector<double> TrajectoryGenerator::trapezoidal_accel(double t_horizon) {
  // unpack current host state
  double s_i = predictor_.host_.s_;
  double v_i = predictor_.host_.s_dot_;
  double a_i = predictor_.host_.s_ddot_;

//  double v_max = std::sqrt( std::pow( COMMON_SPEED_LIMIT, 2 ) - std::pow( predictor_.host_.d_dot_, 2 ) );
//  double a_max = std::sqrt( std::pow( COMMON_MAX_ACCEL, 2 ) - std::pow( predictor_.host_.d_ddot_, 2 ) );
//  double v_max = COMMON_SPEED_LIMIT - COMMON_MAX_SPEED_DELTA;
  double v_max = predictor_.host_.speed_limit_;
  double a_max = COMMON_MAX_ACCEL - COMMON_MAX_ACCEL_DELTA;
  double jerk_max = COMMON_MAX_JERK - COMMON_MAX_JERK_DELTA;

  if ( (predictor_.lane_state_.compare("KL") != 0) | ( abs(predictor_.host_.d_ddot_) > 0.1 ) ) {
    v_max = sqrt( pow(v_max, 2) - pow(2.5, 2) );
    a_max = sqrt( pow(a_max, 2) - pow(1.2, 2) );
    jerk_max = sqrt( pow(jerk_max, 2) - pow(1.5, 2) );
  }

//  if (prev_pts_s_.size() > 0) {
//    // prev_traj_ already initialized
//    v_max = sqrt( pow( COMMON_SPEED_LIMIT, 2 ) - pow( prev_traj_.d_dot_.maxCoeff(), 2 ) );
//    a_max = sqrt( pow( COMMON_MAX_ACCEL, 2 ) - pow( prev_traj_.d_ddot_.maxCoeff(), 2 ) );
//    jerk_max = sqrt( pow( COMMON_MAX_JERK, 2 ) - pow( prev_traj_.d_jerk_.maxCoeff(), 2 ) );
//  }


  // calculate max jerk, trapezoidal acceleration trajectory

  // desired velocity change
  double dv = v_max - v_i;

  // a_i to 0
  double dt_ai0 = a_i / jerk_max;
  double dv_ai0 = a_i * dt_ai0 / 2;

  // a_i to a_max, a_max to a_i
  double dt_aimax = (a_max - a_i) / jerk_max;
  double dv_aimax = (a_max + a_i) * dt_aimax / 2;

  // define state at t_horizon
  double s_f = 0;
  double v_f = 0;
  double a_f = 0;

  if (dv < dv_ai0) {
    // decreasing acceleration

    // segment 1 - decreasing acceleration
    double dt = dt_ai0;
    a_f += a_i
          - jerk_max * dt;
    v_f += v_i
          + a_i * dt
          - 0.5 * jerk_max * pow(dt, 2);
    s_f += s_i
          + v_i * dt
          + 0.5 * a_i * pow(dt, 2)
          - jerk_max * pow(dt, 3) / 6;

    if (t_horizon > dt_ai0) {
      // segment 2 - constant velocity
      dt = t_horizon - dt_ai0;
      a_f = 0;
      v_f = v_max;
      s_f += v_f * dt;
    }

//    cout << "decreasing accel, s_f, v_f, a_f = " << road_.s_norm(s_f) << ", " << v_f << ", " << a_f << endl;

  } else if (dv >= (dv_ai0 + 2 * dv_aimax)) {
    // trapezoidal acceleration
    double dt_amax = (1 / a_max) * (dv - dv_ai0 - 2 * dv_aimax);

    // segment 1 - increasing acceleration
    double dt = min(t_horizon, dt_aimax);
    a_f += a_i
          + jerk_max * dt;
    v_f += v_i
          + a_i * dt
          + 0.5 * jerk_max * pow(dt, 2);
    s_f += s_i
          + v_i * dt
          + 0.5 * a_i * pow(dt, 2)
          + jerk_max * pow(dt, 3) / 6;
    if (t_horizon > dt_aimax) {
      // segment 2 - constant acceleration
      dt = min(t_horizon - dt_aimax, dt_amax);
      s_f += v_f * dt
             + 0.5 * a_f * pow(dt, 2);
      v_f += a_f * dt;
    }

    if (t_horizon > (dt_aimax + dt_amax)) {
      // segment 3 - decreasing acceleration
      dt = min(t_horizon - (dt_aimax + dt_amax), dt_aimax + dt_ai0);
      s_f += v_f * dt
             + 0.5 * a_f * pow(dt, 2)
             - jerk_max * pow(dt, 3) / 6;
      v_f += a_f * dt
             - 0.5 * jerk_max * pow(dt, 2);
      a_f += -jerk_max * dt;
    }

    if (t_horizon > (2 * dt_aimax + dt_amax + dt_ai0)) {
      // segment 4 - constant velocity
      dt = t_horizon - (2 * dt_aimax + dt_amax + dt_ai0);
      a_f = 0;
      v_f = v_max;
      s_f += v_f * dt;
    }

//    cout << "trapezoidal accel, s_f, v_f, a_f = " << road_.s_norm(s_f) << ", " << v_f << ", " << a_f << endl;

  } else {
    // triangular acceleration
    double a_pk = sqrt( jerk_max * (dv - dv_ai0) + pow(a_i, 2) );
    double dt_aipk = (a_pk - a_i) / jerk_max;

    // segment 1 - increasing acceleration
    double dt = min(t_horizon, dt_aipk);
    a_f += a_i
          + jerk_max * dt;
    v_f += v_i
          + a_i * dt
          + 0.5 * jerk_max * pow(dt, 2);
    s_f += s_i
          + v_i * dt
          + 0.5 * a_i * pow(dt, 2)
          + jerk_max * pow(dt, 3) / 6;
    if (t_horizon > dt_aipk) {
      // segment 2 - decreasing acceleration
      dt = min(t_horizon - dt_aipk, dt_aipk + dt_ai0);
      s_f += v_f * dt
             + 0.5 * a_f * pow(dt, 2)
             - jerk_max * pow(dt, 3) / 6;
      v_f += a_f * dt
             - 0.5 * jerk_max * pow(dt, 2);
      a_f += -jerk_max * dt;
    }

    if (t_horizon > (2 * dt_aipk + dt_ai0)) {
      // segment 3 - constant velocity
      dt = t_horizon - (2 * dt_aipk + dt_ai0);
      a_f = 0;
      v_f = v_max;
      s_f += v_f * dt;
    }

//    cout << "triangular accel, s_f, v_f, a_f = " << road_.s_norm(s_f) << ", " << v_f << ", " << a_f << endl;
  }

  s_f = road_.s_norm(s_f);
  return {s_f, v_f, a_f};
}


vector<double> TrajectoryGenerator::trapezoidal_accel(double t_horizon, double v_goal, double a_max, double jerk_max) {
  // unpack current host state
  double s_i = predictor_.host_.s_;
  double v_i = predictor_.host_.s_dot_;
  double a_i = predictor_.host_.s_ddot_;

  // calculate max jerk, trapezoidal acceleration trajectory

  // desired velocity change
  double dv = abs(v_goal - v_i);
  double dv_sign = 0;
  if (dv > 0) {
    dv_sign = (v_goal - v_i) / dv;
  }

  // a_i to 0
  double dt_ai0 = abs(a_i) / jerk_max;
  double dv_ai0 = abs(a_i) * dt_ai0 / 2;

  // a_i to a_max, a_max to a_i
  double dt_aimax = ( a_max - min(abs(a_i), a_max) ) / jerk_max;
  double dv_aimax = ( a_max + min(abs(a_i), a_max) ) * dt_aimax / 2;

  // define state at t_horizon
  double s_f = 0;
  double v_f = 0;
  double a_f = 0;

  double dt_opposite = 0;
  if ( ( (dv_sign > 0) & (a_i < 0) ) | ( (dv_sign < 0) & (a_i > 0) ) ) {
    dt_opposite = dt_ai0;
    dt_ai0 = 0;
    dv_ai0 = 0;
    dt_aimax = a_max / jerk_max;
    dv_aimax = a_max * dt_aimax / 2;
  }

  if (dv < dv_ai0) {
    // decreasing acceleration

    // segment 1 - decreasing acceleration
    double dt = dt_ai0;
    a_f += a_i
          - dv_sign * jerk_max * dt;
    v_f += v_i
          + a_i * dt
          - 0.5 * dv_sign * jerk_max * pow(dt, 2);
    s_f += s_i
          + v_i * dt
          + 0.5 * a_i * pow(dt, 2)
          - dv_sign * jerk_max * pow(dt, 3) / 6;

    if (t_horizon > dt_ai0) {
      // segment 2 - constant velocity
      dt = t_horizon - dt_ai0;
      a_f = 0;
      v_f = v_goal;
      s_f += v_f * dt;
    }

//    cout << "decreasing accel, s_f, v_f, a_f = " << road_.s_norm(s_f) << ", " << v_f << ", " << a_f << endl;

  } else if (dv >= (dv_ai0 + 2 * dv_aimax)) {
    // trapezoidal acceleration
    double dt_amax = (1 / a_max) * (dv - dv_ai0 - 2 * dv_aimax);

    // segment 0 - negative initial acceleration
    double dt = min(t_horizon, dt_opposite);
    if (dt > 0) {
      a_f += dv_sign * jerk_max * dt;
      v_f += v_i
            + a_i * dt
            + (-dv_sign) * 0.5 * jerk_max * pow(dt, 2);
      s_f += s_i
            + v_i * dt
            + 0.5 * a_i * pow(dt, 2)
            + jerk_max * pow(dt, 3) / 6;
    } else {
      a_f += a_i;
      v_f += v_i;
      s_f += s_i;
    }
    if (t_horizon > dt_opposite) {
      // segment 1 - increasing acceleration
      dt = min(t_horizon - dt_opposite, dt_aimax);
      a_f += jerk_max * dt;
      v_f += a_f * dt
            + 0.5 * jerk_max * pow(dt, 2);
      s_f += v_f * dt
            + 0.5 * a_f * pow(dt, 2)
            + jerk_max * pow(dt, 3) / 6;
    }
    if (t_horizon > (dt_opposite + dt_aimax)) {
      // segment 2 - constant acceleration
      dt = min(t_horizon - (dt_opposite + dt_aimax), dt_amax);
      s_f += v_f * dt
             + 0.5 * a_f * pow(dt, 2);
      v_f += a_f * dt;
    }

    if (t_horizon > (dt_opposite + dt_aimax + dt_amax)) {
      // segment 3 - decreasing acceleration
      dt = min(t_horizon - (dt_opposite + dt_aimax + dt_amax), dt_aimax + dt_ai0);
      s_f += v_f * dt
             + 0.5 * a_f * pow(dt, 2)
             - jerk_max * pow(dt, 3) / 6;
      v_f += a_f * dt
             - 0.5 * jerk_max * pow(dt, 2);
      a_f += -jerk_max * dt;
    }

    if (t_horizon > (dt_opposite + 2 * dt_aimax + dt_amax + dt_ai0)) {
      // segment 4 - constant velocity
      dt = t_horizon - (dt_opposite + 2 * dt_aimax + dt_amax + dt_ai0);
      a_f = 0;
      v_f = v_goal;
      s_f += v_f * dt;
    }

//    cout << "trapezoidal accel, s_f, v_f, a_f = " << road_.s_norm(s_f) << ", " << v_f << ", " << a_f << endl;

  } else {
    // triangular acceleration
    double a_pk = sqrt( jerk_max * (dv - dv_ai0) + pow(a_i, 2) );
    double dt_aipk = (a_pk - a_i) / jerk_max;

    // segment 0 - negative initial acceleration
    double dt = min(t_horizon, dt_opposite);
    if (dt > 0) {
      a_f += jerk_max * dt;
      v_f += v_i
            + a_i * dt
            + 0.5 * jerk_max * pow(dt, 2);
      s_f += s_i
            + v_i * dt
            + 0.5 * a_i * pow(dt, 2)
            + jerk_max * pow(dt, 3) / 6;
    } else {
      a_f += a_i;
      v_f += v_i;
      s_f += s_i;
    }
    if (t_horizon > dt_opposite) {
      // segment 1 - increasing acceleration
      double dt = min(t_horizon - dt_opposite, dt_aipk);
      a_f += jerk_max * dt;
      v_f += a_f * dt
            + 0.5 * jerk_max * pow(dt, 2);
      s_f += v_f * dt
            + 0.5 * a_f * pow(dt, 2)
            + jerk_max * pow(dt, 3) / 6;
    }
    if (t_horizon > (dt_opposite + dt_aipk)) {
      // segment 2 - decreasing acceleration
      dt = min(t_horizon - (dt_opposite + dt_aipk), dt_aipk + dt_ai0);
      s_f += v_f * dt
             + 0.5 * a_f * pow(dt, 2)
             - jerk_max * pow(dt, 3) / 6;
      v_f += a_f * dt
             - 0.5 * jerk_max * pow(dt, 2);
      a_f += -jerk_max * dt;
    }
    if (t_horizon > (dt_opposite + 2 * dt_aipk + dt_ai0)) {
      // segment 3 - constant velocity
      dt = t_horizon - (dt_opposite + 2 * dt_aipk + dt_ai0);
      a_f = 0;
      v_f = v_goal;
      s_f += v_f * dt;
    }

//    cout << "triangular accel, s_f, v_f, a_f = " << road_.s_norm(s_f) << ", " << v_f << ", " << a_f << endl;
  }

  s_f = road_.s_norm(s_f);
  return {s_f, v_f, a_f};
}


//vector< vector<double> > TrajectoryGenerator::keep_lane(int lane, double t_horizon, int prev_size) {
vector< vector<double> > TrajectoryGenerator::keep_lane(double t_horizon, int prev_size) {
  int lane = predictor_.desired_lane_;
  // check for lead vehicle
//  int in_front_id = predictor_.in_front(road_.get_lane(predictor_.host_.d_));
  int in_front_id = predictor_.in_path();

  // get host end state following trapezoidal acceleration trajectory
  vector<double> host_end = TrajectoryGenerator::trapezoidal_accel(t_horizon);
  double sh_f = host_end[0];
  double vh_f = host_end[1];
  double ah_f = host_end[2];

  double s1_f = 0;
  double v1_f = 0;
  double a1_f = 0;

  if (in_front_id > -1) {
    // lead vehicle present
    vector<double> lead_now = predictor_.targets_.at(in_front_id).state();
    double s1_i = lead_now[0];
    double v1_i = lead_now[1];
    double a1_i = lead_now[2];
    vector<double> lead_end = predictor_.targets_.at(in_front_id).state_at(t_horizon);
//    double s1_f = lead_end[0];
//    double v1_f = lead_end[1];
//    double a1_f = lead_end[2];
    s1_f = lead_end[0];
    v1_f = lead_end[1];
    a1_f = lead_end[2];

    double range_final = road_.s_diff(s1_f, sh_f);
    double range_now = road_.s_diff(s1_i, predictor_.host_.s_);
    double desired_gap_final = FOLLOWER_T_GAP * vh_f + FOLLOWER_R0;
    double desired_gap_now = FOLLOWER_T_GAP * predictor_.host_.s_dot_ + FOLLOWER_R0;

//    if (range <= (FOLLOWER_T_GAP * v1_f + FOLLOWER_R0)) {
    if ((range_now < desired_gap_now) | (range_final < desired_gap_final)) { // TODO: OR (sh_f + stop_dist) > s1(t_stop)
      std::cout << "desired_gap_now/final = " << desired_gap_now << ", " << desired_gap_final << std::endl;
//      sh_f = road_.s_norm(s1_f - (FOLLOWER_T_GAP * v1_f + FOLLOWER_R0));
      sh_f = road_.s_norm(s1_f - (FOLLOWER_T_GAP * vh_f + FOLLOWER_R0));
      vh_f = min(v1_f, vh_f); // why not eval vh_f before sh_f?
      ah_f = 0;

//      double r_now = road_.s_diff(s1_i, predictor_.host_.s_);
//      double r_dot_now = v1_i - predictor_.host_.s_dot_;
    }
  }

  double dh_f = (lane + 0.5) * predictor_.road_.lane_width_;

  // TODO: hack - cheat inward on far left and far right lanes to avoid bogus "out of lane" fault
  if (lane == 0) {
    dh_f += TRAJECTORY_OUTER_LANE_CHEAT_IN;
  } else if (lane == 2) {
    dh_f -= TRAJECTORY_OUTER_LANE_CHEAT_IN;
  }

  vector<double> goal_state = {sh_f, vh_f, ah_f, dh_f, 0.0, 0.0};

//  cout << "goal_state = ";
//  for (int i = 0; i < goal_state.size(); i++) {
//    cout << goal_state[i] << ", ";
//  }
//  cout << endl;

  return TrajectoryGenerator::get_points_for_goal(goal_state, t_horizon, prev_size);
}


vector< vector<double> > TrajectoryGenerator::get_points_for_goal(vector<double> goal_state,
                                                                  double t_horizon, int prev_size) {
//  vector<double> start_state = predictor_.host_.state();
  vector<double> start_state;
  int pts_used = TRAJECTORY_N_POINTS - prev_size;
  if (prev_pts_s_.size() > 0) {
    int idx_start = pts_used + TRAJECTORY_N_OVERLAP - 1;
    start_state = {prev_pts_s_[idx_start],
                   prev_pts_s_dot_[idx_start],
                   prev_pts_s_ddot_[idx_start],
                   prev_pts_d_[idx_start],
                   prev_pts_d_dot_[idx_start],
                   prev_pts_d_ddot_[idx_start]};
  } else {
    start_state = predictor_.host_.state();
  }
  vector<double> s_start = vector<double>(start_state.begin(), start_state.begin() + 3);
  vector<double> d_start = vector<double>(start_state.begin() + 3, start_state.end());
  vector<double> s_end = vector<double>(goal_state.begin(), goal_state.begin() + 3);
  vector<double> d_end = vector<double>(goal_state.begin() + 3, goal_state.end());

  // handle start s > end s (loop around)
//  cout << "traj gen: s_start, s_goal = " << start_state[0] << ", " << goal_state[0] << endl;
  if ( (s_start[0] > s_end[0]) & ( (road_.max_s_ - s_start[0]) < 100 ) ) { // TODO magic value
    s_end[0] += road_.max_s_;
    goal_state[0] += road_.max_s_;
  }

//  VectorXd s_coef = TrajectoryGenerator::JMT(s_start, s_end, t_horizon);
//  VectorXd d_coef = TrajectoryGenerator::JMT(d_start, d_end, t_horizon);
//  trajectory traj = {s_coef, d_coef, t_horizon};

  trajectory traj = TrajectoryGenerator::PTG(start_state, goal_state, t_horizon);
//  Trajectory traj = TrajectoryGenerator::PTG(start_state, goal_state, t_horizon);

  return TrajectoryGenerator::get_points(traj, prev_size);
}


vector<double> TrajectoryGenerator::perturb_goal(vector<double> end_state, double sigma_multiplier) {
  vector<double> new_state(6);
  // perturb s
  new_state = end_state;
  for (int i = 0; i < 3; i++) {
    normal_distribution<double> dist_s(end_state[i], sigma_multiplier * PTG_SIGMA_S[i]);
    new_state[i] = dist_s(gen_);
  }
  // perturb d - only if changing lanes
  if (predictor_.lane_state_.compare("KL") != 0) {
    for (int i = 0; i < 3; i++) {
      normal_distribution<double> dist_d(end_state[i+3], sigma_multiplier * PTG_SIGMA_D[i]);
      new_state[i+3] = dist_d(gen_);
    }
  }
  return new_state;
}


double TrajectoryGenerator::perturb_t(double T) {
  normal_distribution<double> dist_T(T, PTG_SIGMA_T);
  return dist_T(gen_);
}
