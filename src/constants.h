#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"


// Common
const double COMMON_MAX_JERK = 10; // m/s3
const double COMMON_MAX_ACCEL = 10; // m/s2
const double COMMON_SPEED_LIMIT = 47 / 2.237; // m/s WAS 50
const double COMMON_MAX_JERK_DELTA = 2; // m/s3
const double COMMON_MAX_ACCEL_DELTA = 2; // m/s2
//const double COMMON_MAX_SPEED_DELTA = 4 / 2.237; // m/s2
const double COMMON_LANE_WIDTH = 4; // m
const int COMMON_N_LANES = 3;
const double COMMON_MAX_S = 6945.554; // The max s value (in m) before wrapping around the track back to 0
const double COMMON_IN_PATH_D_DIST = 2.5; // m
const double COMMON_ADJACENT_S_DIST = 3.0; // m
const double COMMON_INF_DIST = 1e6; // m
const double COMMON_EPSILON_DIST = 1e-6; // m

// Trajectory Limits
const double TRAJECTORY_VEHICLE_RADIUS = 1.5; // m
const double TRAJECTORY_EXPECTED_JERK_IN_ONE_SEC = 2; // m/s2
const double TRAJECTORY_EXPECTED_ACCEL_IN_ONE_SEC = 1; // m/s
const double TRAJECTORY_S_BUFFER = 6; // m
const double TRAJECTORY_D_BUFFER = 2.5; // m
const double TRAJECTORY_OUTER_LANE_CHEAT_IN = 0.15; // in m/s
const double TRAJECTORY_LEFT_LIMIT = ( COMMON_LANE_WIDTH / 2 ) + TRAJECTORY_OUTER_LANE_CHEAT_IN;
const double TRAJECTORY_RIGHT_LIMIT = ( COMMON_LANE_WIDTH * ( COMMON_N_LANES - 0.5 ) ) - TRAJECTORY_OUTER_LANE_CHEAT_IN;
const double TRAJECTORY_LANE_CHANGE_VELOCITY_OFFSET = 4; // WAS 2.5
const double TRAJECTORY_LANE_CHANGE_ACCEL_OFFSET = 1.2;
const double TRAJECTORY_LANE_CHANGE_JERK_OFFSET = 1.5;


// Trajectory Weights
const double TRAJECTORY_COLLISION_WEIGHT = 1.1e9;
const double TRAJECTORY_BUFFER_WEIGHT = 3e6; // was 1e5
const double TRAJECTORY_TIME_DIFF_WEIGHT = 100;
const double TRAJECTORY_S_DIFF_WEIGHT = 100;
const double TRAJECTORY_D_DIFF_WEIGHT = 100;
const double TRAJECTORY_MAX_ACCEL_WEIGHT = 1.2e9;
const double TRAJECTORY_TOTAL_ACCEL_WEIGHT = 10;
const double TRAJECTORY_MAX_JERK_WEIGHT = 1.3e9;
const double TRAJECTORY_TOTAL_JERK_WEIGHT = 10;
const double TRAJECTORY_EFFICIENCY_WEIGHT = 1e4;
const double TRAJECTORY_STAYS_ON_ROAD_WEIGHT = 1e6;
const double TRAJECTORY_EXCEEDS_SPEED_LIMIT_WEIGHT = 1.4e9;
const double TRAJECTORY_LATERAL_OFFSET_COST = 1e6;

// Trajectory Times
const int TRAJECTORY_N_EVAL_TIME_STEPS = 40; // number of time steps at which to evaluate trajectory of host and targets WAS 40
const double TRAJECTORY_MAX_EVAL_TIME = 4.0; // for precomputed powers of t

Eigen::MatrixXd inline get_times() {
  Eigen::VectorXd t;
  t.setLinSpaced(TRAJECTORY_N_EVAL_TIME_STEPS, 0.0, TRAJECTORY_MAX_EVAL_TIME);
  Eigen::MatrixXd times = Eigen::MatrixXd(TRAJECTORY_N_EVAL_TIME_STEPS, 6);
  for (int i = 0; i < 6; i++) {
    times.col(i) = t.array().pow(i);
  }
  return times;
}
const Eigen::MatrixXd TRAJECTORY_TIMES = get_times();
const double TRAJECTORY_DT = TRAJECTORY_TIMES(1, 1);

// PTG parameters
const int PTG_N_SAMPLES = 50;
const std::vector<double> PTG_SIGMA_S = {6, 2, 1};
const std::vector<double> PTG_SIGMA_D = {0.25, 0, 0};
const double PTG_SIGMA_T = 0.33; // WAS 0.7
const int PTG_N_TIME_STEPS = 1; // total time steps = 2 * PTG_N_TIME_STEPS + 1
const double PTG_SIGMA_JERK_MAX = 1; // WAS 0.1
const double PTG_MIN_T = 1;

// trajectory parameters
const double TRAJECTORY_TIME_STEP = 0.020; // in seconds
const int TRAJECTORY_N_POINTS = 50; // total number of points passed to simulator
const int TRAJECTORY_N_OVERLAP = 0; // number of points to overlap between current and previous paths

// follower parameters
const double FOLLOWER_T_GAP = 0.5; // in seconds was
const double FOLLOWER_R0 = 6; // longitudinal spacing (in m) for stationary lead vehicle

// lane selection parameters
const int LANE_SELECTOR_START_LANE = 1; // startup lane index
const double LANE_SELECTOR_MIN_REAR_TTC = 3.0; // in seconds WAS 2.0
const double LANE_SELECTOR_MAX_INLANE_OFFSET = 1; // max host lateral offset to centerline (in m) to be
                                                    // considered "in lane" (lane change completed)
const double LANE_SELECTOR_TTC_SECOND_DEGREE_MIN_REL_ACCEL = 1; // min relative acceleration (in m/s2)
                                                                  // for calculating second degree TTC
const double LANE_SELECTOR_TTC_MAX = 1e3; // in seconds
const double LANE_SELECTOR_LATERAL_T_HORIZON = 0; // time (in seconds) at which leading vehicle calculated
const double LANE_SELECTOR_LONGITUDINAL_T_HORIZON = 8; // time (in seconds) at which host advance calculated WAS 4.5
const double LANE_SELECTOR_MIN_PROGRESS_DELTA = 4.5; // an optimal lane must yield at least this much more
                                                     // longitudinal progress (in m) than the current lane
const double LANE_SELECTOR_SPEED_OFFSET = 1; // reduce speed by this amount below adjacent lane vehicle (in m/s) WAS 3
const double LANE_SELECTOR_HYSTERESIS = 2;
const double LANE_SELECTOR_OPTIMAL_LANE_FILTER_TIME = 0.5; // optimal lane must remain unchanged for this time (in seconds)
                                                           // before passing to lane selector WAS 1 - leading vehicle oscillating
                                                           // just below speed limit prevents lane change to open lane
const double LANE_SELECTOR_BUFFER = 5 * TRAJECTORY_VEHICLE_RADIUS; // rear buffer distance (in m) WAS 4

#endif
