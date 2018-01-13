#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <vector>


const double MAX_JERK = 0.9 * 10; // m/s3
const double MAX_ACCEL = 0.8 * 10; // m/s2
const double VEHICLE_RADIUS = 1.5; // m
const double INF_DIST = 1e6; // m
const double EPSILON_DIST = 1e-6; // m

// road parameters
const double SPEED_LIMIT = 45 / 2.237; // m/s
const double LANE_WIDTH = 4; // m
const int N_LANES = 3;
const double MAX_S = 6945.554; // The max s value (in m) before wrapping around the track back to 0

// PTG parameters
const int PTG_N_SAMPLES = 10;
const std::vector<double> PTG_SIGMA_S = {5, 2, 2};
const std::vector<double> PTG_SIGMA_D = {1, 1, 1};
const double PTG_SIGMA_T = 0.5;
const int PTG_N_TIME_STEPS = 2; // total time steps = 2 * PTG_N_TIME_STEPS + 1

// PTG weights
const double PTG_TIME_DIFF_WEIGHT = 1;
const double PTG_S_DIFF_WEIGHT = 1;
const double PTG_D_DIFF_WEIGHT = 1;
const double PTG_COLLISION_WEIGHT = 1;
const double PTG_BUFFER_WEIGHT = 1;
const double PTG_STAYS_ON_ROAD_WEIGHT = 1;
const double PTG_EXCEEDS_SPEED_LIMIT_WEIGHT = 1;
const double PTG_EFFICIENCY_WEIGHT = 10000;
const double PTG_MAX_ACCEL_WEIGHT = 1000;
const double PTG_TOTAL_ACCEL_WEIGHT = 1;
const double PTG_MAX_JERK_WEIGHT = 1000;
const double PTG_TOTAL_JERK_WEIGHT = 1;

// trajectory parameters
const double TRAJECTORY_TIME_STEP = 0.020; // in seconds
const int TRAJECTORY_N_POINTS = 50; // total number of points passed to simulator
const int TRAJECTORY_N_OVERLAP = 0; // number of points to overlap between current and previous paths

// follower parameters
const double FOLLOWER_T_GAP = 0.5; // in seconds
const double FOLLOWER_R0 = 2; // longitudinal spacing (in m) for stationary lead vehicle

// lane selection parameters
const int LANE_SELECTOR_START_LANE = 1; // startup lane index
const double LANE_SELECTOR_MIN_REAR_TTC = 2.0; // in seconds
const double LANE_SELECTOR_MAX_INLANE_OFFSET = 0.5; // max host lateral offset to centerline (in m) to be
                                                    // considered "in lane" (lane change completed)
const double LANE_SELECTOR_TTC_SECOND_DEGREE_MIN_REL_ACCEL = 0.1; // min relative acceleration (in m/s2)
                                                                  // for calculating second degree TTC
const double LANE_SELECTOR_TTC_MAX = 1e3; // in seconds
const double LANE_SELECTOR_LATERAL_T_HORIZON = 0; // time (in seconds) at which leading vehicle calculated
const double LANE_SELECTOR_LONGITUDINAL_T_HORIZON = 3; // time (in seconds) at which host advance calculated
const double LANE_SELECTOR_MIN_PROGRESS_DELTA = 2; // an optimal lane must yield at least this much more
                                                   // longitudinal progress (in m) than the current lane
const double LANE_SELECTOR_BUFFER = 3 * VEHICLE_RADIUS; // front and rear buffer distance (in m)

#endif
