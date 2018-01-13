#include "vehicle.h"

#include <math.h>
#include <iostream>


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
}


Vehicle::~Vehicle() {}


std::vector<double> Vehicle::state() const {
  return std::vector<double> {s_, s_dot_, s_ddot_, d_, d_dot_, d_ddot_};
}


std::vector<double> Vehicle::state_at(double t) const {
  std::vector<double> state(6);
  state[0] = std::max(s_ + ( s_dot_ * t ) + ( s_ddot_ * pow(t, 2) / 2 ), s_ ); // assume vehicle moves forward only
  state[1] = std::max(s_dot_ + s_ddot_ * t, 0.0); // assume vehicle only moves forwards
  state[2] = s_ddot_;
  state[3] = d_ + ( d_dot_ * t ) + ( d_ddot_ * pow(t, 2) / 2 );
  state[4] = d_dot_ + d_ddot_ * t;
  state[5] = d_ddot_;

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
  if (s_dot_prev < 0.001) { // TODO: hack - 12 tgts always returned - if new tgt appears v jumps from 0 to 20+ in single time step
    s_ddot_ = 0;
  } else {
    s_ddot_ = ( s_dot_ - s_dot_prev ) / dt;
  }

  d_ddot_ = ( d_dot_ - d_dot_prev ) / dt;
}


void Vehicle::update_with_accel(std::vector<double> telemetry_comp, double s_ddot, double d_ddot) {
  x_ = telemetry_comp[0];
  y_ = telemetry_comp[1];
  yaw_ = telemetry_comp[2];
  s_ = telemetry_comp[3];
  d_ = telemetry_comp[4];
  s_dot_ = telemetry_comp[5];
  d_dot_ = telemetry_comp[6];

  s_ddot_ = s_ddot;
  d_ddot_ = d_ddot;
}


void Vehicle::print() {
  std::vector<double> state = Vehicle::state();
  std::cout << "s, s_dot, s_ddot, d, d_dot, d_ddot = ";
  for (int i = 0; i < state.size(); i++) {
    std::cout << state[i];
    if (i < (state.size() - 1)) {
      std::cout << ", ";
    }
  }
  std::cout << std::endl;
}
