#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <string>

#include "road.h"
#include "vehicle.h"
#include "predictor.h"


class BehaviorPlanner {
 public:
//  struct collider {
//
//    bool collision ; // is there a collision?
//    int  time; // time collision happens
//
//  };
//
//  int L = 1;
//
//  int preferred_buffer = 6; // impacts "keep lane" behavior.
//
  int lane;

  int s;

  int v;

  int a;

//  int target_speed;
//
//  int lanes_available;
//
  int max_acceleration;

//  int goal_lane;
//
//  int goal_s;
//
  std::string state;

  /**
  * Constructor
  */
  BehaviorPlanner(int lane, int s, int v, int a);

  /**
  * Destructor
  */
  virtual ~BehaviorPlanner();

//  void update_state(map<int, vector <vector<int> > > predictions);
//
//  void configure(vector<int> road_data);
//
//  void increment(int dt);
//
//  vector<int> state_at(int t);
//
//  bool collides_with(Vehicle other, int at_time);
//
//  collider will_collide_with(Vehicle other, int timesteps);
//
//  void realize_state(map<int, vector < vector<int> > > predictions);
//
//  void realize_constant_speed();
//
//  int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);
//
//  void realize_keep_lane(map<int, vector< vector<int> > > predictions);
//
//  void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);
//
//  void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);
//
//  vector<vector<int> > generate_predictions(int horizon);
//
//  // my code
//  struct pose {
//    string state;
//    int lane;
//    int s;
//    int v; // int ok for int timestep
//    int a; // really int?
//  };
//
//  pose get_pose();
//
//  void set_pose(pose pose);
//
//  vector<pose> generate_trajectory(string state, map<int,vector < vector<int> > > predictions, int horizon);
//
//  double compute_cost(vector<pose> trajectory, map<int,vector < vector<int> > > predictions);

};


#endif
