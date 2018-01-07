#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <vector>
#include <chrono>
#include <map>

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

  // list of target vehicles
  std::map<int, Vehicle> targets_;

  // host vehicle
  Vehicle host_ = Vehicle({0, 0, 0, 0, 0, 0, 0});

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
//  void update(double host_x, double host_y, double host_s, double host_d,
//              double host_yaw, double host_speed,
//              std::vector<std::vector <double> > sensor_fusion);

  /**
  * Update host state with Frenet state, x/y, and yaw.
  */
  void update_host(std::vector<double> frenet_state, double x, double y, double yaw);


  /**
  * Get id of first vehicle in front of host in specified lane.
  * Return -1 if no vehicle present.
  */
  int in_front(int lane) const;

  /**
  * Get first vehicle behind host in specified lane.
  * Return -1 if no vehicle present.
  */
  int behind(int lane) const;

 private:
  // road object with map waypoints
  const Road &road_;

  /**
  * Convert target telemetry {id, x, y, vx, vy, s, d} to vehicle
  * state variables {x, y, yaw, s, d, s_dot, d_dot}.
  */
  std::vector<double> compensate_target_telemetry(std::vector<double> target_telemetry) const;

  /**
  * Convert host telemetry to vehicle state variables
  * {x, y, yaw, s, d, s_dot, d_dot}.
  */
//  std::vector<double> compensate_host_telemetry(double x, double y, double s, double d, double yaw, double speed) const;
};


#endif
