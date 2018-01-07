#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <vector>


// Adapted from L5.30 - constants.py
const double MAX_JERK = 0.9 * 10; // m/s3
const double MAX_ACCEL = 0.8 * 10; // m/s2

const double EXPECTED_JERK_IN_ONE_SEC = 2; // m/s2
const double EXPECTED_ACCEL_IN_ONE_SEC = 1; // m/s

const double VEHICLE_RADIUS = 1.5; // m


// road parameters
const double SPEED_LIMIT = 48 / 2.237; // m/s
const double LANE_WIDTH = 4; // m
const double N_LANES = 3;
const double MAX_S = 6945.554; // The max s value (in m) before wrapping around the track back to 0

// PTG parameters
const int PTG_N_SAMPLES = 10;
const vector<double> PTG_SIGMA_S = {5, 2, 2};
const vector<double> PTG_SIGMA_D = {1, 1, 1};
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

const double TRAJECTORY_TIME_STEP = 0.020; // in seconds
const int TRAJECTORY_N_POINTS = 50; // total number of points passed to simulator
const int TRAJECTORY_N_OVERLAP = 0; // number of points to overlap between current and previous paths

// follower parameters
const double FOLLOWER_T_GAP = 0.5; // in seconds
const double FOLLOWER_R0 = 2; // longitudinal spacing (in m) for stationary lead vehicle

#endif
