#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <vector>
#include <chrono>
#include <map>
#include <string>

#include "Eigen-3.3/Eigen/Core"

#include "vehicle.h"
#include "road.h"


typedef std::chrono::time_point<std::chrono::system_clock> systime;

// get current time
inline systime now() {
  return std::chrono::system_clock::now();
}


// get time interval in seconds
inline double interval(systime t_start, systime t_end) {
  std::chrono::duration<double> elapsed = t_end - t_start;
  return elapsed.count();
}


class Predictor {
 public:
  // time stamp of last update
  systime t_update_;

  bool host_init_;

  bool targets_init_;

  // current lane state machine state
  // KL = keep lane
  // LCL = lane change left
  // LCR = lane change right
  // PLCL = prepare lane change left
  // PLCR = prepare lane change right
  std::string lane_state_;

  // previous state
  std::string prev_lane_state_;
  int prev_optimal_lane_;
  int filtered_optimal_lane_;

  // desired lane output by state machine
  int desired_lane_;

  // true is lane change is no longer in progress
  bool lane_change_complete_;

  // time stamp of last entry into keep lane state
  systime t_keep_lane_;

  // time stamp of last optimal lane toggle
  systime t_optimal_lane_;

  // list of target vehicles
  std::map<int, Vehicle> targets_;

  // host vehicle
  Vehicle host_ = Vehicle({0, 0, 0, 0, 0, 0, 0});

  // road object with map waypoints
  const Road &road_;

  /**
  * Constructor
  */
  Predictor(const Road &road);

  /**
  * Destructor
  */
  virtual ~Predictor();

  /**
  * Update with sensor fusion telemetry.
  */
  void update_targets(std::vector< std::vector<double> > sensor_fusion);

  /**
  * Update host state with Frenet state, x/y, and yaw.
  */
  void update_host(std::vector<double> frenet_state, double x, double y, double yaw);

  /**
  * Get id of first vehicle in front of host in specified lane.
  * Return -1 if no vehicle present.
  */
  int in_front(int lane, double t) const;
  int in_front(int lane) const; // at current time

  /**
  * Get id of first vehicle in host path (regardless of lane).
  */
  int in_path(double t) const;
  int in_path() const; // at current time

  /**
  * Get id of first vehicle behind host in specified lane.
  * Return -1 if no vehicle present.
  */
  int behind(int lane, double t) const;
  int behind(int lane) const; // at current time

  /**
  * Get host lane index.
  */
  int host_lane();

  /**
  * Get host lateral offset to center of specified lane.
  */
  double host_lane_offset(int lane);

  /**
  * Calculate optimal lane in terms of forward progress.
  */
  int find_optimal_lane();

  /**
  * Calculate rear TTC for given lane.
  */
  double rear_ttc(int lane);

  /**
  * Select desired lane.
  * Update lane selection state machine.
  */
  void update_lane_selector(double t_horizon, bool abort_lane_change);

  /**
  * Check front and rear buffer in specified lane.
  */
  bool is_safe(int lane, double t);
  bool is_safe(int lane); // at current time

  /**
  * Find speed of target in adjacent lane (within either front or rear buffer).
  */
  double lane_speed(int lane, double t);

  /**
  * Evaluate predicted target s and d coordinates at specified time steps.
  * Output[0] = S(t) MatrixXd (each target is a column, each row is a time step)
  * Output[1] = D(t) MatrixXd (each target is a column, each row is a time step)
  */
  std::vector< Eigen::MatrixXd > target_predictions(double T, int steps);

 private:
  /**
  * Convert target telemetry {id, x, y, vx, vy, s, d} to vehicle
  * state variables {x, y, yaw, s, d, s_dot, d_dot}.
  */
  std::vector<double> compensate_target_telemetry(std::vector<double> target_telemetry) const;

};


#endif
