#include "vehicle.h"

#include <math.h>
#include <iostream>

#include "constants.h"

Vehicle::Vehicle(std::vector<double> telemetry_comp) {
  x_ = telemetry_comp[0];
  y_ = telemetry_comp[1];
  yaw_ = telemetry_comp[2];
  s_ = telemetry_comp[3];
  d_ = telemetry_comp[4];
  s_dot_ = telemetry_comp[5];
  d_dot_ = telemetry_comp[6];

  // init acceleration to 0
  s_ddot_ = 0;
  d_ddot_ = 0;

  // init history vectors
  t_history_.setConstant(VEHICLE_N_POINTS, 1e6);
  s_dot_history_.setZero(VEHICLE_N_POINTS);
  s_ddot_history_.setZero(VEHICLE_N_POINTS);
  d_dot_history_.setZero(VEHICLE_N_POINTS);

  speed_limit_ = COMMON_SPEED_LIMIT;
}


Vehicle::~Vehicle() {}


std::vector<double> Vehicle::state() const {
  return std::vector<double> {s_, s_dot_, s_ddot_, d_, d_dot_, d_ddot_};
}


std::vector<double> Vehicle::state_at(double t) const {
  std::vector<double> state(6);
  double s_t_stop  = 1e6;
  if ((s_ddot_ < 0) && (s_dot_ >= 0)) {
    s_t_stop = -s_dot_ / s_ddot_;
  }
  double s_t_eval = std::min(t, s_t_stop);
  // assume vehicle moves forward only
  state[0] = std::max(s_ + ( s_dot_ * s_t_eval ) +
                      ( s_ddot_ * pow(s_t_eval, 2) / 2 ), s_ );
  state[1] = std::max(s_dot_ + s_ddot_ * s_t_eval, 0.0);
  state[2] = s_ddot_;

  if ( std::abs(mean_d_dot_) < VEHICLE_MIN_MEAN_D_DOT ) {
    // treat weaving adjacent targets as having d_dot = 0
    state[3] = d_;
    state[4] = 0;
    state[5] = 0;
  } else {
    // assume constant d_dot
    state[3] = d_ + ( d_dot_ * t );
    state[4] = d_dot_;
    state[5] = 0;
  }

  return state;
}


std::vector<double> Vehicle::min_s_ddot_state_at(double t) const {
  std::vector<double> state(6);
  double s_t_stop  = 1e6;
  if ((min_s_ddot_ < 0) && (s_dot_ >= 0)) {
    s_t_stop = -s_dot_ / min_s_ddot_;
  }
  double s_t_eval = std::min(t, s_t_stop);
  // assume vehicle moves forward only
  state[0] = std::max(s_ + ( s_dot_ * s_t_eval ) +
                      ( min_s_ddot_ * pow(s_t_eval, 2) / 2 ), s_ );
  state[1] = std::max(s_dot_ + min_s_ddot_ * s_t_eval, 0.0);
  state[2] = min_s_ddot_;

  if ( std::abs(mean_d_dot_) < VEHICLE_MIN_MEAN_D_DOT ) {
    // treat weaving adjacent targets as having d_dot = 0
    state[3] = d_;
    state[4] = 0;
    state[5] = 0;
  } else {
    // assume constant d_dot
    state[3] = d_ + ( d_dot_ * t );
    state[4] = d_dot_;
    state[5] = 0;
  }

  return state;
}



void Vehicle::update(std::vector<double> telemetry_comp, double dt)
{
  // store previous velocity
  double s_dot_prev = s_dot_;
  double d_dot_prev = d_dot_;

  x_ = telemetry_comp[0];
  y_ = telemetry_comp[1];
  yaw_ = telemetry_comp[2];
  s_ = telemetry_comp[3];
  d_ = telemetry_comp[4];
  s_dot_ = telemetry_comp[5];
  d_dot_ = telemetry_comp[6];

  // update acceleration
  double s_ddot = (s_dot_ - s_dot_prev) / dt;
  bool reset = std::abs(s_ddot) > (1.5 * COMMON_MAX_ACCEL);
  if (s_dot_prev < 0.001) {
    // if new tgt appears v jumps from 0 to 20+ in single time step
    s_ddot_ = 0;
  } else {
    s_ddot_ = ( s_dot_ - s_dot_prev ) / dt;
  }

  d_ddot_ = ( d_dot_ - d_dot_prev ) / dt;

  update_history(s_dot_, d_dot_, s_ddot_, d_ddot_, dt, reset);
}


void Vehicle::update_with_accel(std::vector<double> telemetry_comp,
                                double s_ddot, double d_ddot) {
  x_ = telemetry_comp[0];
  y_ = telemetry_comp[1];
  yaw_ = telemetry_comp[2];
  s_ = telemetry_comp[3];
  d_ = telemetry_comp[4];
  s_dot_ = telemetry_comp[5];
  d_dot_ = telemetry_comp[6];

  s_ddot_ = s_ddot;
  d_ddot_ = d_ddot;

  min_s_ddot_ = s_ddot_;
  mean_d_dot_ = d_dot_;
}


void Vehicle::update_history(double s_dot, double d_dot, double s_ddot,
                             double d_ddot, double dt, bool reset) {
  const int n = VEHICLE_N_POINTS - 1;
  if (reset) {
    t_history_.setConstant(VEHICLE_N_POINTS, 1e6);
    s_dot_history_.setZero(VEHICLE_N_POINTS);
    s_ddot_history_.setZero(VEHICLE_N_POINTS);
    d_dot_history_.setZero(VEHICLE_N_POINTS);
  } else {
    s_dot_history_.tail<n>() = s_dot_history_.head<n>();
    s_ddot_history_.tail<n>() = s_ddot_history_.head<n>();
    d_dot_history_.tail<n>() = d_dot_history_.head<n>();
    t_history_.tail<n>() = t_history_.head<n>().array() + dt;
  }

  s_dot_history_(0) = s_dot;
  s_ddot_history_(0) = s_ddot;
  d_dot_history_(0) = d_dot;
  t_history_(0) = 0;

  // update min_s_ddot_
  int s_idx = VEHICLE_N_POINTS - 1;
  while (t_history_(s_idx) > VEHICLE_T_S_DDOT) {
    s_idx--;
  }
  min_s_ddot_ = s_ddot_history_.head(s_idx + 1).minCoeff();

  // update mean_d_dot_
  int d_idx = VEHICLE_N_POINTS - 1;
  while (t_history_(d_idx) > VEHICLE_T_D_DOT) {
    d_idx--;
  }
  mean_d_dot_ = d_dot_history_.head(d_idx + 1).mean();
}


void Vehicle::print() {
  std::vector<double> state = Vehicle::state();
  std::cout << "vehicle: s, s_dot, s_ddot, d, d_dot, d_ddot = ";
  for (int i = 0; i < state.size(); i++) {
    std::cout << state[i];
    if (i < (state.size() - 1)) {
      std::cout << ", ";
    }
  }
  std::cout << std::endl;
}
