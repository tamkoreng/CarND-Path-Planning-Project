# CarND-Path-Planning-Project
## Performance
The implemented path planner works reasonably well in the majority of highway situations encountered in the simulator. The vehicle has been able to run for up to 2 hours without incident while averaging just over 45 mph.

## Code Review
### Behavioral Planning
A simple state machine is implemented in the `Predictor` method [update_lane_selector](src/predictor.cpp#L431). The host lane selection state is one of the following:
* Keep Lane (KL)
* Lane Change Left (LCL)
* Lane Change Right (LCR)
* Prepare Lane Change Left (PLCL)
* Prepare Lane Change Right (PLCR)  

The state transitions are based on the current optimal lane and the availability of free space in the adjacent lane.
* Optimal Lane - The lane in which the host will advance the greatest distance in a given time horizon (currently set at 3 seconds) is treated as optimal. This selection function is implemented in the `Predictor` method [find_optimal_lane](src/predictor.cpp#L280).
* Front Space - The nearest leading adjacent vehicle must be at least `t_gap * s_dot + R0`, where `t_gap` is the desired headway or time gap (currently set at 0.5 s) and `R0` is a fixed distance (currently set at 6 m). This gap is checked by the `Predictor` method [is_safe](src/predictor.cpp#L377).
* Rear Space - The nearest trailing adjacent vehicle must have a TTC (time to collision) exceeding a fixed threshold (currently set at 5.0 s) and must exceed a minimum distance (currently set at 10 m). The TTC is calculated by the `Predictor` method [rear_ttc](src/predictor.cpp#L341).

This `update_lane_selector` method also accepts an `abort_lane_change` argument from the `TrajectoryGenerator` to force a return to the original lane if a collision is predicted while attempting a lane change.

### Target Predictions
Target position predictions assume constant longitudinal acceleration `s_ddot` and constant lateral velocity `d_dot`. The vectorized position calculations are performed by the `Predictor` method  [target_predictions](src/predictor.cpp#L624), and the results are cached for use in the polynomial trajectory generator. To prevent weaving adjacent targets from leading to false collision predictions, `d_dot` is set to 0 if its mean value over a previous time window (currently set at 3.0 s) is below a threshold (currently set at 3.0 m/s). To prevent collisions with leading vehicles with oscillating `s_ddot`, `s_ddot` is set to its minimum value over a previous time window (current set at 3.0 s) if the following distance is below the desired set point.

### Coordinate System
All path planning is implemented in Frenet coordinates. Target positions are converted **from** Cartesian using the `Road` method [get_frenet](src/road.cpp#L81). Host trajectory points are converted **to** Cartesian using splines by the `Road` method [get_xy](src/road.cpp#L124).

### Trajectory Generation
The lane selected by the behavioral planner is realized as a trajectory by the `TrajectoryGenerator` method [keep_lane](src/trajectory_generator.cpp). A trajectory is computed for a fixed time horizon (currently set at 3.0 s) according to the following process:
#### 1. Trapezoidal acceleration profiles  
An initial host goal state (`s`, `s_dot`, `s_ddot`, `d`, `d_dot`, `d_ddot` is created using the `TrajectoryGenerator` method [trapezoidal_accel](src/trajectory_generator.cpp#L293). If there is no leading vehicle, the trajectory end point (goal state) is computed for a trapezoidal acceleration profile at the maximum allowed acceleration and jerk. If a leading vehicle is present, the goal state is taken to be the leading vehicle state less the desired following distance.
#### 2. Jerk minimizing trajectories (JMT)
The proposed goal state is used to instantiate a `Trajectory` object. The jerk minimal fifth degree polynomials `s(t)` and `d(t)` are calculated using the `Trajectory` method [JMT](src/trajectory.cpp#L150). A set of cost functions are then evaluated for the trajectory using the `Trajectory` method [calculate_cost](src/trajectory_generator.cpp#L43). If the cost is below a given threshold, the trajectory is passed to the point generator (Step 4). Otherwise, a set of perturbed trajectories are created (Step 3).
#### 3. Polynomial trajectory generation (PTG)
The `TrajectorGenerator` method [PTG](src/trajectory_generator.cpp#L36) selects a minimal cost trajectory from a set of 50 to 150 perturbed trajectories. Each trajectory is created by randomly perturbing the time horizon `T`, the goal longitudinal velocity `s_dot`, the maximum acceleration, and the maximum jerk and then feeding these perturbed parameters to [trapezoidal_accel](src/trajectory_generator.cpp#L293) using `TrajectoryGenerator` method [perturb_goal](src/trajectory_generator.cpp#L611). If the lowest cost trajectory from the first set of 50 still exceeds the maximum cost threshold, another set is generated using parameter distributions with larger standard deviations. This process repeats up to a maximum number of iterations (currently set at 3).
#### 4. Creating trajectory points
The selected trajectory is finally passed to the `TrajectoryGenerator` method [get_points](src/trajectory_generator.cpp#L134) to generate the path points to feed to the simulator. Points from the previous time step are cached so that the new trajectory starting point is the same as the final point used by the simulator.

***   

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
