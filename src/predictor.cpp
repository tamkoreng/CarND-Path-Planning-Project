#include "predictor.h"

#include <math.h>
#include <iostream>

#include "helpers.h"


using namespace std;


Predictor::Predictor(const Road &road) : road_(road) {
  host_init_ = false;
  targets_init_ = false;
}


Predictor::~Predictor() {}


void Predictor::update_targets(vector< vector <double> > sensor_fusion) {
//void Predictor::update(double host_x, double host_y,
//                       double host_s, double host_d,
//                       double host_yaw, double host_speed,
//                       std::vector<std::vector <double> > sensor_fusion) {
//  // compensate host telemetry
//  vector<double> host_telem = Predictor::compensate_host_telemetry(host_x,
//                                host_y, host_s, host_d, host_yaw, host_speed);

  if (!targets_init_) {
    t_update_ = now();
//    // initialize host
//    host_ = Vehicle(host_telem);

    // initialize targets
    for (int i = 0; i < sensor_fusion.size(); i++) {
      int id = sensor_fusion[i][0];
      vector<double> tgt_telem = Predictor::compensate_target_telemetry(
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

//    // update host
//    host_.update(host_telem, dt);

    // tag existing targets for removal
    map<int, Vehicle>::iterator it;
    map<int, bool> keep;
    it = targets_.begin();
    while (it != targets_.end()) {
      keep.insert({it->first, false});
      ++it;
    }

    for (int i = 0; i < sensor_fusion.size(); i++) {
      int id = sensor_fusion[i][0];
      vector<double> tgt_telem = Predictor::compensate_target_telemetry(
                                   sensor_fusion[i]);
      it = targets_.find(id);
      if (it != targets_.end()) {
        // target already in map - update
//        cout << "tgt " << id << ": v = " << tgt_telem[5] << ", dt = " << dt << endl;
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
    map<int, bool>::iterator it_keep;
    it_keep = keep.begin();
    while (it_keep != keep.end()) {
      if (it_keep->second == false) {
        targets_.erase(it_keep->first);
      }
      ++it_keep;
    }
  }
}


vector<double> Predictor::compensate_target_telemetry(vector<double> telemetry) const {
  // unpack telemetry vector
  double x = telemetry[1];
  double y = telemetry[2];
  double vx = telemetry[3];
  double vy = telemetry[4];
  double s = telemetry[5];
  double d = telemetry[6];

  // compute yaw angle
  double yaw = safe_atan2(vy, vx);

  // convert velocity to Frenet coordinates
  vector<double> frenet_vel = road_.get_frenet_velocity(x, y, vx, vy);
  double s_dot = frenet_vel[0];
  double d_dot = frenet_vel[1];

  // build output vector
  vector<double> telemetry_comp = {x, y, yaw, s, d, s_dot, d_dot};

  return telemetry_comp;
}


void Predictor::update_host(vector<double> frenet_state, double x, double y, double yaw) {
  vector<double> telemetry_comp = {x, y, yaw, frenet_state[0], frenet_state[3],
                                   frenet_state[1], frenet_state[4]};
  host_.update_with_accel(telemetry_comp, frenet_state[2], frenet_state[5]);

  if (!host_init_) {
    host_init_ = true;
  }
}

//vector<double> Predictor::compensate_host_telemetry(double x, double y,
//    double s, double d,
//    double yaw, double speed) const {
//  // convert velocity to Frenet coordinates
//  double vx = speed * cos(yaw);
//  double vy = speed * sin(yaw);
//  vector<double> frenet_vel = road_.get_frenet_velocity(x, y, vx, vy);
//  double s_dot = frenet_vel[0];
//  double d_dot = frenet_vel[1];
//
//  // build output vector
//  vector<double> telemetry_comp = {x, y, yaw, s, d, s_dot, d_dot};
//
//  return telemetry_comp;
//}


int Predictor::in_front(int lane) const {
  auto it = targets_.begin();
  vector<int> ids;

  // get vehicles in front of host in specified lane
  while (it != targets_.end()) {
    int id = it->first;
//    if ((road_.get_lane(it->second.d_) == lane) & (it->second.s_ > host_.s_)) {
//      ids.push_back(id);
//    }
    if (road_.get_lane(it->second.d_) == lane) {
      double delta_s = road_.s_diff(it->second.s_, host_.s_);
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
      double delta_s = road_.s_diff(targets_.at(ids[i]).s_, host_.s_);
      if (delta_s < min_s) {
        min_s = delta_s;
        leading = ids[i];
      }
    }
  }

  return leading;
}


int Predictor::behind(int lane) const {
  auto it = targets_.begin();
  vector<int> ids;

  // get vehicles behind host in specified lane
  while (it != targets_.end()) {
    int id = it->first;
//    if ((road_.get_lane(it->second.d_) == lane) & (it->second.s_ <= host_.s_)) {
//      ids.push_back(id);
//    }
    if (road_.get_lane(it->second.d_) == lane) {
      double delta_s = road_.s_diff(it->second.s_, host_.s_);
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
      double delta_s = road_.s_diff(targets_.at(ids[i]).s_, host_.s_);
      if (delta_s > max_s) {
        max_s = delta_s;
        trailing = ids[i];
      }
    }
  }

  return trailing;
}
