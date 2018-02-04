#include "predictor.h"

#include <math.h>
#include <iostream>

#include "Eigen-3.3/Eigen/Dense"

#include "helpers.h"
#include "constants.h"


Predictor::Predictor(const Road &road) : road_(road) {
  host_init_ = false;
  targets_init_ = false;
  lane_state_ = "KL";
  prev_lane_state_ = "KL";
  t_keep_lane_ = now();
  desired_lane_ = LANE_SELECTOR_START_LANE;
  prev_optimal_lane_ = desired_lane_;
  filtered_optimal_lane_ = desired_lane_;
  t_optimal_lane_ = now();
  lane_change_complete_ = true;
}


Predictor::~Predictor() {}


void Predictor::update_targets(
    std::vector< std::vector <double> > sensor_fusion) {
  if (!targets_init_) {
    t_update_ = now();

    // initialize targets
    for (int i = 0; i < sensor_fusion.size(); i++) {
      int id = sensor_fusion[i][0];
      std::vector<double> tgt_telem = Predictor::compensate_target_telemetry(
                                   sensor_fusion[i]);
      Vehicle new_target = Vehicle(tgt_telem);
      targets_.insert({id, new_target});
    }

    targets_init_ = true;

  } else {
    // update time
    systime t_prev = t_update_;
    t_update_ = now();
    double dt = interval(t_prev, t_update_);

    // tag existing targets for removal
    std::map<int, Vehicle>::iterator it;
    std::map<int, bool> keep;
    it = targets_.begin();
    while (it != targets_.end()) {
      keep.insert({it->first, false});
      ++it;
    }

    for (int i = 0; i < sensor_fusion.size(); i++) {
      int id = sensor_fusion[i][0];
      std::vector<double> tgt_telem = Predictor::compensate_target_telemetry(
                                   sensor_fusion[i]);
      it = targets_.find(id);
      if (it != targets_.end()) {
        // target already in std::map - update
        it->second.update(tgt_telem, dt);

        // don't remove this target
        keep[it->first] = true;
      } else {
        // new target - insert
        Vehicle new_target = Vehicle(tgt_telem);
        targets_.insert({id, new_target});
      }
    }

    // remove targets not in current telemetry
    std::map<int, bool>::iterator it_keep;
    it_keep = keep.begin();
    while (it_keep != keep.end()) {
      if (it_keep->second == false) {
        targets_.erase(it_keep->first);
      }
      ++it_keep;
    }
  }
}


std::vector<double> Predictor::compensate_target_telemetry(
    std::vector<double> telemetry) const {
  // unpack telemetry std::vector
  double x = telemetry[1];
  double y = telemetry[2];
  double vx = telemetry[3];
  double vy = telemetry[4];
  double s = telemetry[5];
  double d = telemetry[6];

  // compute yaw angle
  double yaw = safe_atan2(vy, vx);

  // convert velocity to Frenet coordinates
  std::vector<double> frenet_vel = road_.get_frenet_velocity(x, y, vx, vy);
  double s_dot = frenet_vel[0];
  double d_dot = frenet_vel[1];

  // build output std::vector
  std::vector<double> telemetry_comp = {x, y, yaw, s, d, s_dot, d_dot};

  return telemetry_comp;
}


void Predictor::update_host(std::vector<double> frenet_state, double x,
    double y, double yaw) {
  std::vector<double> telemetry_comp = {x, y, yaw, frenet_state[0],
                                        frenet_state[3], frenet_state[1],
                                        frenet_state[4]};
  host_.update_with_accel(telemetry_comp, frenet_state[2], frenet_state[5]);

  if (!host_init_) {
    host_init_ = true;
  }
}


int Predictor::in_front(int lane, double t) const {
  std::vector<double> host_at_t = host_.state_at(t);
  double host_s_at_t = host_at_t[0];

  auto it = targets_.begin();
  std::vector<int> ids;

  // get vehicles in front of host in specified lane
  while (it != targets_.end()) {
    int id = it->first;
    std::vector<double> target_at_t = it->second.state_at(t);
    double target_s_at_t = target_at_t[0];
    double target_d_at_t = target_at_t[3];
    if (road_.get_lane(target_d_at_t) == lane) {
      double delta_s = road_.s_diff(target_s_at_t, host_s_at_t);
      if (delta_s >= 0) {
        ids.push_back(id);
      }
    }
    it++;
  }

  // get closest vehicle
  int leading = -1;
  if(ids.size() > 0) {
    double min_s = 1000;
    for(int i = 0; i < ids.size(); i++) {
      std::vector<double> target_at_t = targets_.at(ids[i]).state_at(t);
      double target_s_at_t = target_at_t[0];
      double delta_s = road_.s_diff(target_s_at_t, host_s_at_t);
      if (delta_s < min_s) {
        min_s = delta_s;
        leading = ids[i];
      }
    }
  }

  return leading;
}


int Predictor::in_front(int lane) const {
  return Predictor::in_front(lane, 0);
}


int Predictor::in_path(double t) const {
  std::vector<double> host_at_t = host_.state_at(t);
  double host_s_at_t = host_at_t[0];
  double host_d_at_t = host_at_t[3];

  auto it = targets_.begin();
  std::vector<int> ids;

  // get vehicles in front of host in specified lane
  while (it != targets_.end()) {
    int id = it->first;
    std::vector<double> target_at_t = it->second.state_at(t);
    double target_s_at_t = target_at_t[0];
    double target_d_at_t = target_at_t[3];
    double delta_d = std::abs( host_d_at_t - target_d_at_t );
    if (delta_d < COMMON_IN_PATH_D_DIST) {
      double delta_s = road_.s_diff(target_s_at_t, host_s_at_t);
      if (delta_s >= 0) {
        ids.push_back(id);
      }
    }
    it++;
  }

  // get closest vehicle
  int leading = -1;
  if(ids.size() > 0) {
    double min_s = 1000;
    for(int i = 0; i < ids.size(); i++) {
      std::vector<double> target_at_t = targets_.at(ids[i]).state_at(t);
      double target_s_at_t = target_at_t[0];
      double delta_s = road_.s_diff(target_s_at_t, host_s_at_t);
      if (delta_s < min_s) {
        min_s = delta_s;
        leading = ids[i];
      }
    }
  }

  return leading;
}


int Predictor::in_path() const {
  return Predictor::in_path(0);
}


int Predictor::behind(int lane, double t) const {
  std::vector<double> host_at_t = host_.state_at(t);
  double host_s_at_t = host_at_t[0];

  auto it = targets_.begin();
  std::vector<int> ids;

  // get vehicles behind host in specified lane
  while (it != targets_.end()) {
    int id = it->first;
    std::vector<double> target_at_t = it->second.state_at(t);
    double target_s_at_t = target_at_t[0];
    double target_d_at_t = target_at_t[3];
    if (road_.get_lane(target_d_at_t) == lane) {
      double delta_s = road_.s_diff(target_s_at_t, host_s_at_t);
      if (delta_s < 0) {
        ids.push_back(id);
      }
    }
    it++;
  }

  // get closest vehicle
  int trailing = -1;
  if(ids.size() > 0) {
    double max_s = -1000;
    for(int i = 0; i < ids.size(); i++) {
      std::vector<double> target_at_t = targets_.at(ids[i]).state_at(t);
      double target_s_at_t = target_at_t[0];
      double delta_s = road_.s_diff(target_s_at_t, host_s_at_t);
      if (delta_s > max_s) {
        max_s = delta_s;
        trailing = ids[i];
      }
    }
  }

  return trailing;
}


int Predictor::behind(int lane) const {
  return Predictor::behind(lane, 0);
}


int Predictor::host_lane() {
  return road_.get_lane(host_.d_);
}


double Predictor::host_lane_offset(int lane) {
  double lane_center = (lane + 0.5) * road_.lane_width_;
  return host_.d_ - lane_center;
}


int Predictor::find_optimal_lane() {
  int lane = 0;

  // calculate how far the host could advance with max acceleration (assume
  // infinite jerk for simplicity)
  double t_to_max_speed = std::min(
      (COMMON_SPEED_LIMIT - host_.s_dot_) / COMMON_MAX_ACCEL,
      LANE_SELECTOR_LONGITUDINAL_T_HORIZON );
  double t_at_max_speed = LANE_SELECTOR_LONGITUDINAL_T_HORIZON - t_to_max_speed;
  double max_accel_dist = (host_.s_dot_ * t_to_max_speed) +
                          ( 0.5 * COMMON_MAX_ACCEL * std::pow(t_to_max_speed, 2) ) +
                          (COMMON_SPEED_LIMIT * t_at_max_speed);

  std::vector<double> host_progress_by_lane;
  double best_progress = 0;
  double best_gap = 0;
  double best_lane = 0;
  while (lane < COMMON_N_LANES) {
    int leading_id = Predictor::in_front(lane, LANE_SELECTOR_LATERAL_T_HORIZON);
    double host_progress;
    double gap;
    if (leading_id > -1) {
      std::vector<double> target_at_t = targets_.at(leading_id).state_at(
          LANE_SELECTOR_LONGITUDINAL_T_HORIZON);
      // relative to CURRENT host position
      double target_s_at_t = road_.s_diff(target_at_t[0], host_.s_);
      double target_s_dot_at_t = target_at_t[1];
      double buffer_dist = ( target_s_dot_at_t * FOLLOWER_T_GAP ) + FOLLOWER_R0;

      gap = std::max(target_s_at_t - buffer_dist, 0.0);
      host_progress = std::min(gap, max_accel_dist);
    } else {
      gap = COMMON_INF_DIST;
      host_progress = max_accel_dist;
    }

    host_progress_by_lane.push_back(host_progress);

    bool is_best = (host_progress > best_progress) ||
                   ( ( std::abs(host_progress - best_progress) < COMMON_EPSILON_DIST ) &&
                     (gap > best_gap) );

    if (is_best) {
      best_progress = host_progress;
      best_gap = gap;
      best_lane = lane;
    }

    lane++;
  }

  int current_lane = Predictor::host_lane();
  double current_lane_progress = host_progress_by_lane[current_lane];
  if (best_progress >= (current_lane_progress + LANE_SELECTOR_MIN_PROGRESS_DELTA) ) {
    return best_lane;
  } else {
    return current_lane;
  }
}


double Predictor::rear_ttc(int lane) {
  double ttc = LANE_SELECTOR_TTC_MAX;
  int trailing_id = Predictor::behind(lane);
  if (trailing_id > -1) {
    // host relative target
    double delta_s = road_.s_diff(host_.s_, targets_.at(trailing_id).s_);
    double delta_s_dot = host_.s_dot_ - targets_.at(trailing_id).s_dot_;
    double delta_s_ddot = host_.s_ddot_ - targets_.at(trailing_id).s_ddot_;

    if (delta_s_ddot >= LANE_SELECTOR_TTC_SECOND_DEGREE_MIN_REL_ACCEL) {
      // second degree solution
      // 0.5 * delta_s_ddot * t^2 + delta_s_dot * t + delta_s = 0
      if ( std::pow(delta_s, 2) > (2 * delta_s_ddot * delta_s) ) {
        double t_plus = ( -delta_s_dot + std::sqrt( std::pow(delta_s_dot, 2) -
                          (2 * delta_s_ddot * delta_s) ) ) / delta_s_ddot;
        double t_minus = ( -delta_s_dot - std::sqrt( std::pow(delta_s_dot, 2) -
                           (2 * delta_s_ddot * delta_s) ) ) / delta_s_ddot;
        if ( (t_plus >= 0) && (t_minus < 0) ) {
          ttc = std::min(t_plus, LANE_SELECTOR_TTC_MAX);
        } else if ( (t_plus < 0) && (t_minus >= 0) ) {
          ttc = std::min(t_minus, LANE_SELECTOR_TTC_MAX);
        } else if ( (t_plus >= 0) && (t_minus >= 0) ) {
          ttc = std::min( std::min(t_plus, t_minus), LANE_SELECTOR_TTC_MAX );
        }
      }
    } else if ( (delta_s_dot < 0) &&
                (delta_s_ddot < LANE_SELECTOR_TTC_SECOND_DEGREE_MIN_REL_ACCEL) ) {
      // first degree solution
      ttc = -delta_s / delta_s_dot;
    }

  }
  return ttc;
}


bool Predictor::is_safe(int lane, double t) {
  // check front buffer distance
  int leading_id = Predictor::in_front(lane, t);
  double delta_s_front = COMMON_INF_DIST;
  if (leading_id > -1) {
    delta_s_front = road_.s_diff( targets_.at(leading_id).state_at(t)[0],
                                 host_.state_at(t)[0] );
  }

  // check rear buffer distance
  int trailing_id = Predictor::behind(lane);
  double delta_s_rear = COMMON_INF_DIST;
  if (trailing_id > -1) {
    delta_s_rear = road_.s_diff( host_.state_at(t)[0],
                                targets_.at(trailing_id).state_at(t)[0] );
  }

  double front_buffer = (host_.state_at(t)[1] * FOLLOWER_T_GAP ) + FOLLOWER_R0;
  bool is_safe = (delta_s_front >= front_buffer ) &&
                 (delta_s_rear >= LANE_SELECTOR_BUFFER);
  return is_safe;
}


double Predictor::lane_speed(int lane, double t) {
  double speed = COMMON_INF_DIST;
  // check front buffer distance
  int leading_id = Predictor::in_front(lane, t);
  double delta_s_front = COMMON_INF_DIST;
  if (leading_id > -1) {
    delta_s_front = road_.s_diff( targets_.at(leading_id).state_at(t)[0],
                                 host_.state_at(t)[0] );
  }

  // check rear buffer distance
  int trailing_id = Predictor::behind(lane);
  double delta_s_rear = COMMON_INF_DIST;
  if (trailing_id > -1) {
    delta_s_rear = road_.s_diff( host_.state_at(t)[0],
                                 targets_.at(trailing_id).state_at(t)[0] );
  }

  double front_buffer = (host_.state_at(t)[1] * FOLLOWER_T_GAP ) + FOLLOWER_R0;
  if (delta_s_rear < LANE_SELECTOR_BUFFER) {
    speed = targets_.at(trailing_id).state_at(t)[1];
  } else if (delta_s_front < front_buffer) {
    speed = targets_.at(leading_id).state_at(t)[1];
  }

  return speed;
}


//void Predictor::update_lane_selector(double t_horizon) {
void Predictor::update_lane_selector(double t_horizon, bool abort_lane_change) {
  int current_lane = Predictor::host_lane();

  // for debug only
  double delta_s_front = COMMON_INF_DIST;
  int in_path_id = in_path();
  if (in_path_id > -1) {
    delta_s_front = road_.s_diff(targets_.at(in_path_id).s_, host_.s_);
  }

  // filter optimal lane toggling
  int new_optimal_lane = Predictor::find_optimal_lane();
  if ( new_optimal_lane != prev_optimal_lane_ ) {
    t_optimal_lane_ = now();
  }
  prev_optimal_lane_ = new_optimal_lane;
  double time_since_optimal_toggle = interval(t_optimal_lane_, now());
  if ( time_since_optimal_toggle >= LANE_SELECTOR_OPTIMAL_LANE_FILTER_TIME ) {
      filtered_optimal_lane_ = new_optimal_lane;
  }
  int optimal_lane = filtered_optimal_lane_;

  // update lane change complete
  double offset_to_desired = std::abs( Predictor::host_lane_offset(desired_lane_) );
  lane_change_complete_ = (current_lane == desired_lane_) &&
                          (offset_to_desired < LANE_SELECTOR_MAX_INLANE_OFFSET);

  // update state
  std::string next_state = lane_state_;
  if (lane_state_.compare("KL") == 0) {
    if (lane_state_.compare( prev_lane_state_ ) != 0) {
      // on entry
      t_keep_lane_ = now();
    }
    double time_in_lane = interval(t_keep_lane_, now());
    bool can_change = (time_in_lane >= LANE_SELECTOR_HYSTERESIS) &&
                      lane_change_complete_;
    host_.speed_limit_ = COMMON_SPEED_LIMIT;

    if (COMMON_DEBUG) {
      std::cout << "KL: current = " << current_lane << ", optimal = "
                << optimal_lane << ", delta_s_front = " << delta_s_front
                <<  ", time_in_lane = " << time_in_lane << std::endl;
    }

    if ( (optimal_lane < current_lane) && can_change ) {
      int proposed_lane = current_lane - 1;
      bool is_safe_start = is_safe(proposed_lane, 0) &&
                           (rear_ttc(proposed_lane) >= LANE_SELECTOR_MIN_REAR_TTC);
      bool is_safe_end = is_safe(proposed_lane, t_horizon);

      if (is_safe_start && is_safe_end) {
        next_state = "LCL";
        desired_lane_ = proposed_lane;
        lane_change_complete_ = false;
      } else if (optimal_lane < proposed_lane) {
        next_state = "PLCL";
      } else {
        desired_lane_ = current_lane;
      }
    } else if ( (optimal_lane > current_lane) && can_change ) {
      int proposed_lane = current_lane + 1;
      bool is_safe_start = is_safe(proposed_lane, 0) &&
                           (rear_ttc(proposed_lane) >= LANE_SELECTOR_MIN_REAR_TTC);
      bool is_safe_end = is_safe(proposed_lane, t_horizon);

      if (is_safe_start && is_safe_end) {
        next_state = "LCR";
        desired_lane_ = proposed_lane;
        lane_change_complete_ = false;
      } else if (optimal_lane > proposed_lane) {
        next_state = "PLCR";
      } else {
        desired_lane_ = current_lane;
      }
    }
  } else if (lane_state_.compare("LCL") == 0) {

    if (COMMON_DEBUG) {
      std::cout << "LCL: current = " << current_lane << ", optimal = "
                << optimal_lane << ", delta_s_front = " << delta_s_front
                << std::endl;
    }

    if (lane_change_complete_) {
      next_state = "KL";
    } else {
      if (abort_lane_change) {
        int proposed_lane = desired_lane_ + 1;
        bool is_safe_start = is_safe(proposed_lane, 0) &&
                             (rear_ttc(proposed_lane) >= LANE_SELECTOR_MIN_REAR_TTC);
        bool is_safe_end = is_safe(proposed_lane, t_horizon);
        if (is_safe_start && is_safe_end) {
          next_state = "KL";
          desired_lane_ = proposed_lane;

          if (COMMON_DEBUG) {
            std::cout << "Aborting lane change - returning to lane "
                      << desired_lane_ << ", delta_s_front = " << delta_s_front
                      <<std::endl;
          }
        }
      }
    }
  } else if (lane_state_.compare("LCR") == 0) {

    if (COMMON_DEBUG) {
      std::cout << "LCR: current = " << current_lane << ", optimal = "
                << optimal_lane << ", delta_s_front = " << delta_s_front
                <<std::endl;
    }

    if (lane_change_complete_) {
      next_state = "KL";
    } else {
      if (abort_lane_change) {
        int proposed_lane = desired_lane_ - 1;
        bool is_safe_start = is_safe(proposed_lane, 0) &&
                             (rear_ttc(proposed_lane) >= LANE_SELECTOR_MIN_REAR_TTC);
        bool is_safe_end = is_safe(proposed_lane, t_horizon);
        if (is_safe_start && is_safe_end) {
          next_state = "KL";
          desired_lane_ = proposed_lane;

          if (COMMON_DEBUG) {
            std::cout << "Aborting lane change - returning to lane "
                      << desired_lane_ << ", delta_s_front = " << delta_s_front
                      << std::endl;
          }
        }
      }
    }
  } else if (lane_state_.compare("PLCL") == 0) {
    int proposed_lane = current_lane - 1;
    double max_speed = std::min( lane_speed(proposed_lane, 0),
                                lane_speed(proposed_lane, t_horizon) );
    double desired_speed = std::max( max_speed - LANE_SELECTOR_SPEED_OFFSET, 0.0 );
    bool safe = is_safe(proposed_lane, 0) &&
                is_safe(proposed_lane, t_horizon) &&
                (rear_ttc(proposed_lane) >= LANE_SELECTOR_MIN_REAR_TTC);

    if (COMMON_DEBUG) {
      std::cout << "PLCL: current = " << current_lane << ", optimal = "
                << optimal_lane << ", delta_s_front = " << delta_s_front
                << ", desired_speed = " << desired_speed << std::endl;
    }

    if (optimal_lane < (current_lane - 1)) {
      if ( (host_.s_dot_ < max_speed) && safe ) {
        next_state = "LCL";
        desired_lane_ = proposed_lane;
        lane_change_complete_ = false;
      } else {
        next_state = "PLCL";
        host_.speed_limit_ = std::min( desired_speed, COMMON_SPEED_LIMIT );
      }
    } else {
      next_state = "KL";
    }
  } else if (lane_state_.compare("PLCR") == 0) {
    int proposed_lane = current_lane + 1;
    double max_speed = std::min( lane_speed(proposed_lane, 0),
                                lane_speed(proposed_lane, t_horizon) );
    double desired_speed = std::max( max_speed - LANE_SELECTOR_SPEED_OFFSET, 0.0 );
    bool safe = is_safe(proposed_lane, 0) &&
                is_safe(proposed_lane, t_horizon) &&
                (rear_ttc(proposed_lane) >= LANE_SELECTOR_MIN_REAR_TTC);

    if (COMMON_DEBUG) {
      std::cout << "PLCR: current = " << current_lane << ", optimal = "
                << optimal_lane << ", delta_s_front = " << delta_s_front
                << ", desired_speed = " << desired_speed << std::endl;
    }

    if (optimal_lane > (current_lane + 1)) {
      if ( (host_.s_dot_ < max_speed) && safe ) {
        next_state = "LCR";
        desired_lane_ = proposed_lane;
        lane_change_complete_ = false;
      } else {
        next_state = "PLCR";
        host_.speed_limit_ = std::min( desired_speed, COMMON_SPEED_LIMIT );
      }
    } else {
      next_state = "KL";
    }
  }

  prev_lane_state_ = lane_state_;
  lane_state_ = next_state;
}


std::vector< Eigen::MatrixXd > Predictor::target_predictions(double T,
                                                             int steps) {
  int n_tgt = targets_.size();

  Eigen::VectorXd t;
  t.setLinSpaced(steps, 0.0, T);
  double dt = T / steps;

  Eigen::VectorXd ones;
  ones.setOnes(steps);

  Eigen::RowVectorXd s = Eigen::RowVectorXd(n_tgt);
  Eigen::RowVectorXd s_dot = Eigen::RowVectorXd(n_tgt);
  Eigen::RowVectorXd s_ddot = Eigen::RowVectorXd(n_tgt);
  Eigen::RowVectorXd d = Eigen::RowVectorXd(n_tgt);
  Eigen::RowVectorXd d_dot = Eigen::RowVectorXd(n_tgt);
  Eigen::RowVectorXd d_ddot = Eigen::RowVectorXd(n_tgt);

  Eigen::MatrixXd S = Eigen::MatrixXd(steps, n_tgt);
  Eigen::MatrixXd D = Eigen::MatrixXd(steps, n_tgt);

  auto it = targets_.begin();
  std::vector<int> ids;

  while (it != targets_.end()) {
    int i = it->first;
    s(i) = it->second.s_;
    s_dot(i) = it->second.s_dot_;
    s_ddot(i) = it->second.s_ddot_;
    d(i) = it->second.d_;
    d_dot(i) = it->second.d_dot_;
    d_ddot(i) = it->second.d_ddot_;
    ++it;
  }

  Eigen::VectorXd t_squared = t.array().pow(2);
  S = ( ones * s ) + ( t * s_dot ) + ( 0.5 * t_squared * s_ddot );

  // ensure S < max_s
  auto loop_back = S.array() > COMMON_MAX_S;
  Eigen::MatrixXd S_filtered = loop_back.select( S.array() - COMMON_MAX_S, S );

  // ensure s_dot > 0
  Eigen::RowVectorXd s_max = S_filtered.colwise().maxCoeff();
  Eigen::MatrixXd S_max = ones * s_max;
  auto negative_s_dot = S_filtered.array() < S_max.array();
  Eigen::MatrixXd S_valid = negative_s_dot.select( S_filtered, S_max );

  D = ( ones * d ) + ( t * d_dot ); // ignoring d_ddot

  return {S, D};
}
