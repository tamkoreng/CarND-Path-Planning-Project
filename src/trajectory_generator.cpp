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


TrajectoryGenerator::TrajectoryGenerator(const Road &road,
    const Predictor &predictor) : road_(road), predictor_(predictor) {
  gen_ = make_seeded_engine();
}


TrajectoryGenerator::~TrajectoryGenerator() {}


//trajectory TrajectoryGenerator::PTG(vector<double> start_state, vector<double> end_state, double T) {
//  double time_step = PTG_SIGMA_T / PTG_N_TIME_STEPS;
//  vector< vector<double> > end_states;
//  vector<double> times;
//  double t = max(T - PTG_N_TIME_STEPS * time_step, TRAJECTORY_TIME_STEP);
//
//  // generate end states
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
//
//  // generate jerk minimal trajectories
//  vector<trajectory> trajectories;
//  vector<double> costs;
//  for (int i = 0; i < end_states.size(); i++) {
//    vector<double> s_start = vector<double>(start_state.begin(),
//                                            start_state.begin() + 3);
//    vector<double> d_start = vector<double>(start_state.begin() + 3,
//                                            start_state.end());
//    vector<double> s_end = vector<double>(end_states[i].begin(),
//                                          end_states[i].begin() + 3);
//    vector<double> d_end = vector<double>(end_states[i].begin() + 3,
//                                          end_states[i].end());
//
//    VectorXd s_coef = JMT(s_start, s_end, times[i]);
//    VectorXd d_coef = JMT(d_start, d_end, times[i]);
//
//    trajectory traj = {s_coef, d_coef, times[i]};
//    trajectories.push_back(traj);
//    costs.push_back(calculate_cost(traj, start_state, end_state, T));
//  }
//
//  // find minimum cost trajectory
//  int min_idx = distance( costs.begin(), min_element(costs.begin(), costs.end()) );
//  return trajectories[min_idx];
//}


vector< vector<double> > TrajectoryGenerator::get_points(trajectory traj, int prev_size) {
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
    // first time called
    t_max = TRAJECTORY_N_POINTS * TRAJECTORY_TIME_STEP;

    // use host state for start state
//    traj = TrajectoryGenerator::PTG(predictor_.host_.state(), end_state, T);
    start_state = predictor_.host_.state();
  }

//  // use JMT to generate new points
//  vector<double> s_start = vector<double>(start_state.begin(), start_state.begin() + 3);
//  vector<double> d_start = vector<double>(start_state.begin() + 3, start_state.end());
//  vector<double> s_end = vector<double>(end_state.begin(), end_state.begin() + 3);
//  vector<double> d_end = vector<double>(end_state.begin() + 3, end_state.end());
//  cout << "s_start[1], s_end[1] = " << s_start[1] << ", " << s_end[1] << endl;
//  VectorXd s_coef = TrajectoryGenerator::JMT(s_start, s_end, T);
//  VectorXd d_coef = TrajectoryGenerator::JMT(d_start, d_end, T);
//  trajectory traj = {s_coef, d_coef, T};

//  cout << "s_coef = ";
//  for (int i = 0; i < s_coef.size(); i++) {
//    cout << s_coef[i] << ", ";
//  }
//  cout << endl;

  // get polynomials for new trajectory derivatives
  VectorXd s_dot_poly = polyder(traj.s_coef);
  VectorXd s_ddot_poly = polyder(s_dot_poly);
  VectorXd d_dot_poly = polyder(traj.d_coef);
  VectorXd d_ddot_poly = polyder(d_dot_poly);

  double t = TRAJECTORY_TIME_STEP; // t = 0 @ idx_start
  while (t < (min(t_max, traj.T) + t_epsilon)) {
    // evaluate trajectory polynomials to get points
    new_pts_s[idx] = road_.s_norm(polyeval(traj.s_coef, t));
    new_pts_s_dot[idx] = polyeval(s_dot_poly, t);
    new_pts_s_ddot[idx] = polyeval(s_ddot_poly, t);
    new_pts_d[idx] = polyeval(traj.d_coef, t);
    new_pts_d_dot[idx] = polyeval(d_dot_poly, t);
    new_pts_d_ddot[idx] = polyeval(d_ddot_poly, t);

    // convert to x/y coordinates
    vector<double> next_pt = road_.get_xy(new_pts_s[idx], new_pts_d[idx]);
    new_pts_x[idx] = next_pt[0];
    new_pts_y[idx] = next_pt[1];

    t += TRAJECTORY_TIME_STEP;
    idx++;
  }

  if (traj.T < t_max) {
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

//  cout << "get_points: exited" << endl;
  return {new_pts_x, new_pts_y};
}


vector<double> TrajectoryGenerator::get_host_state(int prev_size) {
  int idx = TRAJECTORY_N_POINTS - prev_size - 1;
//  cout << "get_host_state - idx = " << idx << endl;
//  cout << "get_host_state - prev_pts_s_.size() = " << prev_pts_s_.size() << endl;
//  cout << "get_host_state - prev_pts_s_[0] = " << prev_pts_s_[0] << endl;
//  cout << "get_host_state - prev_pts_s_dot_[0] = " << prev_pts_s_dot_[0] << endl;
//  cout << "get_host_state - prev_pts_s_ddot_[0] = " << prev_pts_s_ddot_[0] << endl;
//  cout << "get_host_state - prev_pts_d_[0] = " << prev_pts_d_[0] << endl;
//  cout << "get_host_state - prev_pts_d_dot_[0] = " << prev_pts_d_dot_[0] << endl;
//  cout << "get_host_state - prev_pts_d_ddot_[0] = " << prev_pts_d_ddot_[0] << endl;
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
  cout << "trapezoidal_accel: entered" << endl;
  // unpack current host state
  double s_i = predictor_.host_.s_;
  double v_i = predictor_.host_.s_dot_;
  double a_i = predictor_.host_.s_ddot_;

  // calculate max jerk, trapezoidal acceleration trajectory

  // desired velocity change
  double dv = SPEED_LIMIT - v_i;
  cout << "dv = " << dv << endl;

  // a_i to 0
  double dt_ai0 = a_i / MAX_JERK;
  double dv_ai0 = a_i * dt_ai0 / 2;
  cout << "a_i to 0 - dv, dt = " << dv_ai0 << ", " << dt_ai0 << endl;

  // a_i to a_max, a_max to a_i
  double dt_aimax = (MAX_ACCEL - a_i) / MAX_JERK;
  double dv_aimax = (MAX_ACCEL + a_i) * dt_aimax / 2;
  cout << "a_i to max - dv, dt = " << dv_aimax << ", " << dt_aimax << endl;

  // define state at t_horizon
  double s_f = 0;
  double v_f = 0;
  double a_f = 0;

  if (dv < dv_ai0) {
    // decreasing acceleration
    cout << "trapezoidal_accel: decreasing acceleration" << endl;

    // segment 1 - decreasing acceleration
    double dt = dt_ai0;
    a_f += a_i
          - MAX_JERK * dt;
    v_f += v_i
          + a_i * dt
          - 0.5 * MAX_JERK * pow(dt, 2);
    s_f += s_i
          + v_i * dt
          + 0.5 * a_i * pow(dt, 2)
          - MAX_JERK * pow(dt, 3) / 6;
    cout << "dt, s_f, v_f, a_f = " << dt << ", " << s_f << ", " << v_f << ", " << a_f << endl;

    if (t_horizon > dt_ai0) {
      // segment 2 - constant velocity
      dt = t_horizon - dt_ai0;
      a_f = 0;
      v_f = SPEED_LIMIT;
      s_f += v_f * dt;
      cout << "dt, s_f, v_f, a_f = " << dt << ", " << s_f << ", " << v_f << ", " << a_f << endl;
    }

  } else if (dv >= (dv_ai0 + 2 * dv_aimax)) {
    // trapezoidal acceleration
    cout << "trapezoidal_accel: trap accel segment 1" << endl;
    double dt_amax = (1 / MAX_ACCEL) * (dv - dv_ai0 - 2 * dv_aimax);
    cout << "dt_amax = " << dt_amax << endl;

    // segment 1 - increasing acceleration
    double dt = min(t_horizon, dt_aimax);
    a_f += a_i
          + MAX_JERK * dt;
    v_f += v_i
          + a_i * dt
          + 0.5 * MAX_JERK * pow(dt, 2);
    s_f += s_i
          + v_i * dt
          + 0.5 * a_i * pow(dt, 2)
          + MAX_JERK * pow(dt, 3) / 6;
    cout << "dt, s_f, v_f, a_f = " << dt << ", " << s_f << ", " << v_f << ", " << a_f << endl;
    if (t_horizon > dt_aimax) {
      // segment 2 - constant acceleration
      cout << "trapezoidal_accel: trap accel segment 2" << endl;
      dt = min(t_horizon - dt_aimax, dt_amax);
      s_f += v_f * dt
             + 0.5 * a_f * pow(dt, 2);
      v_f += a_f * dt;
      cout << "dt, s_f, v_f, a_f = " << dt << ", " << s_f << ", " << v_f << ", " << a_f << endl;
    }

    if (t_horizon > (dt_aimax + dt_amax)) {
      // segment 3 - decreasing acceleration
      cout << "trapezoidal_accel: trap accel segment 3" << endl;
      dt = min(t_horizon - (dt_aimax + dt_amax), dt_aimax + dt_ai0);
      s_f += v_f * dt
             + 0.5 * a_f * pow(dt, 2)
             - MAX_JERK * pow(dt, 3) / 6;
      v_f += a_f * dt
             - 0.5 * MAX_JERK * pow(dt, 2);
      a_f += -MAX_JERK * dt;
      cout << "dt, s_f, v_f, a_f = " << dt << ", " << s_f << ", " << v_f << ", " << a_f << endl;
    }

    if (t_horizon > (2 * dt_aimax + dt_amax + dt_ai0)) {
      // segment 4 - constant velocity
      cout << "trapezoidal accel: trap accel segment 4" << endl;
      dt = t_horizon - (2 * dt_aimax + dt_amax + dt_ai0);
      a_f = 0;
      v_f = SPEED_LIMIT;
      s_f += v_f * dt;
      cout << "dt, s_f, v_f, a_f = " << dt << ", " << s_f << ", " << v_f << ", " << a_f << endl;
    }

  } else {
    // triangular acceleration
    cout << "trapezoidal_accel: triangle accel segment 1" << endl;
    double a_pk = sqrt( MAX_JERK * (dv - dv_ai0) + pow(a_i, 2) );
    double dt_aipk = (a_pk - a_i) / MAX_JERK;
    cout << "dt_aipk, a_pk = " << dt_aipk << ", " << a_pk << endl;

    // segment 1 - increasing acceleration
    double dt = min(t_horizon, dt_aipk);
    a_f += a_i
          + MAX_JERK * dt;
    v_f += v_i
          + a_i * dt
          + 0.5 * MAX_JERK * pow(dt, 2);
    s_f += s_i
          + v_i * dt
          + 0.5 * a_i * pow(dt, 2)
          + MAX_JERK * pow(dt, 3) / 6;
    cout << "dt, s_f, v_f, a_f = " << dt << ", " << s_f << ", " << v_f << ", " << a_f << endl;
    if (t_horizon > dt_aipk) {
      // segment 2 - decreasing acceleration
      cout << "trapezoidal_accel: triangle accel segment 2" << endl;
      dt = min(t_horizon - dt_aipk, dt_aipk + dt_ai0);
      s_f += v_f * dt
             + 0.5 * a_f * pow(dt, 2)
             - MAX_JERK * pow(dt, 3) / 6;
      v_f += a_f * dt
             - 0.5 * MAX_JERK * pow(dt, 2);
      a_f += -MAX_JERK * dt;
      cout << "dt, s_f, v_f, a_f = " << dt << ", " << s_f << ", " << v_f << ", " << a_f << endl;
    }

    if (t_horizon > (2 * dt_aipk + dt_ai0)) {
      // segment 3 - constant velocity
      cout << "trapezoidal_accel: triangle accel segment 3" << endl;
      dt = t_horizon - (2 * dt_aipk + dt_ai0);
      a_f = 0;
      v_f = SPEED_LIMIT;
      s_f += v_f * dt;
      cout << "dt, s_f, v_f, a_f = " << dt << ", " << s_f << ", " << v_f << ", " << a_f << endl;
    }
  }

//  cout << "s_f, v_f, a_f = " << s_f << ", " << v_f << ", " << a_f << endl;
  s_f = road_.s_norm(s_f);
  return {s_f, v_f, a_f};
}


vector< vector<double> > TrajectoryGenerator::keep_lane(double t_horizon, int prev_size) {
  cout << "keep_lane: entered" << endl;
  // check for lead vehicle
  int in_front_id = predictor_.in_front(road_.get_lane(predictor_.host_.d_));

  // get host end state following trapezoidal acceleration trajectory
  vector<double> host_end = TrajectoryGenerator::trapezoidal_accel(t_horizon);
  double sh_f = host_end[0];
  double vh_f = host_end[1];
  double ah_f = host_end[2];
//  cout << "sh_f, vh_f, ah_f = " << sh_f << ", " << vh_f << ", " << ah_f << endl;
  cout << "in_front_id = " << in_front_id << endl;

  if (in_front_id > -1) {
    // lead vehicle present
    vector<double> lead_now = predictor_.targets_.at(in_front_id).state();
    double s1_i = lead_now[0];
    double v1_i = lead_now[1];
    double a1_i = lead_now[2];
    vector<double> lead_end = predictor_.targets_.at(in_front_id).state_at(t_horizon);
    double s1_f = lead_end[0];
    double v1_f = lead_end[1];
    double a1_f = lead_end[2];

    cout << "s1_i, v1_i, a1_i = " << s1_i << ", " << v1_i << ", " << a1_i << endl;
    cout << "s1_f, v1_f, a1_f = " << s1_f << ", " << v1_f << ", " << a1_f << endl;

    double range = road_.s_diff(s1_f, sh_f);

    if (range <= (FOLLOWER_T_GAP * v1_f + FOLLOWER_R0)) {
      // TODO: need to watch wrap around
      sh_f = road_.s_norm(s1_f - (FOLLOWER_T_GAP * v1_f + FOLLOWER_R0));
//      if (sh_f < predictor_.host_.s_) {
//        sh_f = road_.s_norm(s1_f - FOLLOWER_R0);
//      }
      vh_f = min(v1_f, SPEED_LIMIT);
//      ah_f = min(a1_f, MAX_ACCEL);
      ah_f = 0;

      double r_now = road_.s_diff(s1_i, predictor_.host_.s_);
      double r_dot_now = v1_i - predictor_.host_.s_dot_;
      cout << "TTC = " << -r_now / r_dot_now << endl;
    }
  }

  vector<double> goal_state = {sh_f, vh_f, ah_f, predictor_.host_.d_, 0.0, 0.0};

  cout << "goal_state = ";
  for (int i = 0; i < goal_state.size(); i++) {
    cout << goal_state[i] << ", ";
  }
  cout << endl;

//  return TrajectoryGenerator::get_points(goal_state, t_horizon, prev_size);
  // use JMT to generate new points
  vector<double> start_state = predictor_.host_.state();
  vector<double> s_start = vector<double>(start_state.begin(), start_state.begin() + 3);
  vector<double> d_start = vector<double>(start_state.begin() + 3, start_state.end());
  vector<double> s_end = vector<double>(goal_state.begin(), goal_state.begin() + 3);
  vector<double> d_end = vector<double>(goal_state.begin() + 3, goal_state.end());
  cout << "s_start[1], s_end[1] = " << s_start[1] << ", " << s_end[1] << endl;

  // handle start s > end s (loop around)
  if (s_start[0] > s_end[0]) {
    s_end[0] += road_.max_s_;
  }

  VectorXd s_coef = TrajectoryGenerator::JMT(s_start, s_end, t_horizon);
  VectorXd d_coef = TrajectoryGenerator::JMT(d_start, d_end, t_horizon);
  trajectory traj = {s_coef, d_coef, t_horizon};

//  trajectory traj = TrajectoryGenerator::PTG(start_state, goal_state, t_horizon);

  return TrajectoryGenerator::get_points(traj, prev_size);
}


VectorXd TrajectoryGenerator::JMT(vector<double> start, vector<double> end, double T) const{
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


//vector<double> TrajectoryGenerator::perturb_goal(vector<double> end_state) {
//  vector<double> new_state(6);
//  // perturb s
//  new_state = end_state;
//  for (int i = 0; i < 3; i++) {
//    normal_distribution<double> dist_s(end_state[i], PTG_SIGMA_S[i]);
//    new_state[i] = dist_s(gen_);
//  }
////  // perturb d
////  for (int i = 0; i < 3; i++) {
////    normal_distribution<double> dist_d(end_state[i+3], PTG_SIGMA_D[i]);
////    new_state[i+3] = dist_d(gen_);
////  }
//  return new_state;
//}
//
//
//double TrajectoryGenerator::calculate_cost(trajectory traj, vector<double> start_state,
//                                           vector<double> end_state, double T) const {
//  double cost = 0;
//  cost += PTG_BUFFER_WEIGHT               * buffer_cost(traj, start_state, end_state, T);
//  cost += PTG_COLLISION_WEIGHT            * collision_cost(traj, start_state, end_state, T);
//  cost += PTG_D_DIFF_WEIGHT               * d_diff_cost(traj, start_state, end_state, T);
//  cost += PTG_EFFICIENCY_WEIGHT           * efficiency_cost(traj, start_state, end_state, T);
//  cost += PTG_EXCEEDS_SPEED_LIMIT_WEIGHT  * exceeds_speed_limit_cost(traj, start_state, end_state, T);
//  cost += PTG_MAX_ACCEL_WEIGHT            * max_accel_cost(traj, start_state, end_state, T);
//  cost += PTG_MAX_JERK_WEIGHT             * max_jerk_cost(traj, start_state, end_state, T);
//  cost += PTG_STAYS_ON_ROAD_WEIGHT        * stays_on_road_cost(traj, start_state, end_state, T);
//  cost += PTG_S_DIFF_WEIGHT               * s_diff_cost(traj, start_state, end_state, T);
//  cost += PTG_TIME_DIFF_WEIGHT            * time_diff_cost(traj, start_state, end_state, T);
//  cost += PTG_TOTAL_ACCEL_WEIGHT          * total_accel_cost(traj, start_state, end_state, T);
//  cost += PTG_TOTAL_JERK_WEIGHT           * total_jerk_cost(traj, start_state, end_state, T);
//
////  cout << "TrajectoryGenerator::calculate_cost - cost = " << cost << endl;
//
//  return cost;
//}
//
//
//double TrajectoryGenerator::nearest_approach(trajectory traj, const Vehicle &veh) const {
//  double closest = 1e6;
////  const int steps = 100;
//  const int steps = 25;
//  for (int i = 0; i < steps; i++) {
//    double t = i / steps * traj.T;
//    double s = polyeval(traj.s_coef, t);
//    double d = polyeval(traj.d_coef, t);
//    vector<double> end_state = veh.state_at(t);
//    double dist = distance_2d(s, d, end_state[0], end_state[3]); // assuming small curvature
//    if (dist < closest) {
//      closest = dist;
//    }
//  }
//
//  return closest;
//}
//
//
//double TrajectoryGenerator::nearest_approach_to_any_vehicle(trajectory traj) const {
//  double closest = 1e6;
//  auto it = predictor_.targets_.begin();
//  while (it != predictor_.targets_.end()) {
//    double dist = nearest_approach(traj, it->second);
//    if (dist < closest) {
//      closest = dist;
//    }
//    it++;
//  }
//
//  return closest;
//}
//
//
//double TrajectoryGenerator::time_diff_cost(trajectory traj, vector<double> start_state,
//                                           vector<double> end_state, double T) const {
//  return logistic(fabs(traj.T - T) / T);
//}
//
//
//double TrajectoryGenerator::s_diff_cost(trajectory traj, vector<double> start_state,
//                                           vector<double> end_state, double T) const {
//  VectorXd s_coef = traj.s_coef;
//  VectorXd s_dot_coef = polyder(s_coef);
//  VectorXd s_ddot_coef = polyder(s_dot_coef);
//  vector<double> actual(3);
//  actual[0] = polyeval(s_coef, traj.T);
//  actual[1] = polyeval(s_dot_coef, traj.T);
//  actual[2] = polyeval(s_ddot_coef, traj.T);
//
//  double cost = 0;
//  for (int i = 0; i < 3; i++) {
//    double diff = fabs(actual[i] - end_state[i]);
//    cost += logistic(diff / PTG_SIGMA_S[i]);
//  }
//
//  return cost;
//}
//
//
//double TrajectoryGenerator::d_diff_cost(trajectory traj, vector<double> start_state,
//                                           vector<double> end_state, double T) const {
//  VectorXd d_coef = traj.d_coef;
//  VectorXd d_dot_coef = polyder(d_coef);
//  VectorXd d_ddot_coef = polyder(d_dot_coef);
//  vector<double> actual(3);
//  actual[0] = polyeval(d_coef, traj.T);
//  actual[1] = polyeval(d_dot_coef, traj.T);
//  actual[2] = polyeval(d_ddot_coef, traj.T);
//
//  double cost = 0;
//  for (int i = 0; i < 3; i++) {
//    double diff = fabs(actual[i] - end_state[i+3]);
//    cost += logistic(diff / PTG_SIGMA_D[i]);
//  }
//
//  return cost;
//}
//
//
//double TrajectoryGenerator::collision_cost(trajectory traj, vector<double> start_state,
//                                           vector<double> end_state, double T) const {
//  double nearest = nearest_approach_to_any_vehicle(traj);
//  double cost = 0;
//  if (nearest < (2 * VEHICLE_RADIUS)) {
//    cost = 1;
//  }
//
//  return cost;
//}
//
//
//double TrajectoryGenerator::buffer_cost(trajectory traj, vector<double> start_state,
//                                        vector<double> end_state, double T) const {
//  double nearest = nearest_approach_to_any_vehicle(traj);
//
//  return logistic( 2 * VEHICLE_RADIUS / nearest );
//}
//
//
//double TrajectoryGenerator::stays_on_road_cost(trajectory traj, vector<double> start_state,
//                                               vector<double> end_state, double T) const {
//  return 0;
//}
//
//
//double TrajectoryGenerator::exceeds_speed_limit_cost(trajectory traj, vector<double> start_state,
//                                                     vector<double> end_state, double T) const {
//  return 0;
//}
//
//
//double TrajectoryGenerator::efficiency_cost(trajectory traj, vector<double> start_state,
//                                            vector<double> end_state, double T) const {
//  double avg_v = polyeval(traj.s_coef, traj.T) / traj.T;
//  double tgt_s = end_state[0];
//  double tgt_avg_v = tgt_s / traj.T;
//
//  return logistic( 2 * ( tgt_avg_v - avg_v ) / avg_v );
//}
//
//
//double TrajectoryGenerator::max_accel_cost(trajectory traj, vector<double> start_state,
//                                           vector<double> end_state, double T) const {
//  const int steps = 100;
//
//  VectorXd s_coef = traj.s_coef;
//  VectorXd s_dot_coef = polyder(s_coef);
//  VectorXd s_ddot_coef = polyder(s_dot_coef);
//
//  double dt = traj.T / steps; // bt - was T - pretty sure it should be traj.T
//  vector<double> all_accels(steps);
//  for (int i = 0; i < steps; i++) {
//    double t = i * dt;
//    all_accels[i] = polyeval(s_ddot_coef, t);
//  }
//
//  double max_accel = *max_element(all_accels.begin(), all_accels.end());
//  double cost = 0;
//  if (max_accel > MAX_ACCEL) {
//    cost = 1;
//  }
//
//  return cost;
//}
//
//
//double TrajectoryGenerator::total_accel_cost(trajectory traj, vector<double> start_state,
//                                             vector<double> end_state, double T) const {
//  const int steps = 100;
//
//  VectorXd s_coef = traj.s_coef;
//  VectorXd s_dot_coef = polyder(s_coef);
//  VectorXd s_ddot_coef = polyder(s_dot_coef);
//
//  double dt = traj.T / steps; // bt - was T - pretty sure it should be traj.T
//  double total_accel = 0;
//  for (int i = 0; i < steps; i++) {
//    double t = i * dt;
//    double accel = polyeval(s_ddot_coef, t);
//    total_accel += fabs(accel * dt);
//  }
//
//  double accel_per_sec = total_accel / traj.T; // bt - was T - pretty sure it should be traj.T
//
//  return logistic( accel_per_sec / EXPECTED_ACCEL_IN_ONE_SEC );
//}
//
//
//double TrajectoryGenerator::max_jerk_cost(trajectory traj, vector<double> start_state,
//                                          vector<double> end_state, double T) const {
//  const int steps = 100;
//
//  VectorXd s_coef = traj.s_coef;
//  VectorXd s_dot_coef = polyder(s_coef);
//  VectorXd s_ddot_coef = polyder(s_dot_coef);
//  VectorXd s_jerk_coef = polyder(s_ddot_coef);
//
//  double dt = traj.T / steps; // bt - was T - pretty sure it should be traj.T
//  vector<double> all_jerks(steps);
//  for (int i = 0; i < steps; i++) {
//    double t = i * dt;
//    all_jerks[i] = polyeval(s_jerk_coef, t);
//  }
//
//  double max_jerk = *max_element(all_jerks.begin(), all_jerks.end());
//  double cost = 0;
//  if (max_jerk > MAX_JERK) {
//    cost = 1;
//  }
//
//  return cost;
//}
//
//
//double TrajectoryGenerator::total_jerk_cost(trajectory traj, vector<double> start_state,
//                                            vector<double> end_state, double T) const {
//  const int steps = 100;
//
//  VectorXd s_coef = traj.s_coef;
//  VectorXd s_dot_coef = polyder(s_coef);
//  VectorXd s_ddot_coef = polyder(s_dot_coef);
//  VectorXd s_jerk_coef = polyder(s_ddot_coef);
//
//  double dt = traj.T / steps; // bt - was T - pretty sure it should be traj.T
//  double total_jerk = 0;
//  for (int i = 0; i < steps; i++) {
//    double t = i * dt;
//    double jerk = polyeval(s_jerk_coef, t);
//    total_jerk += fabs(jerk * dt);
//  }
//
//  double jerk_per_sec = total_jerk / traj.T; // bt - was T - pretty sure it should be traj.T
//
//  return logistic( jerk_per_sec / EXPECTED_JERK_IN_ONE_SEC );
//}
