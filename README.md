# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Reflections

Path Planning tries to replicate the thinking and decision making the humans do while driving, and decide the optimal action based on safety, efficiency, legality and confort. This project implements a simple version of this process, in a very concrete environment: a 3 lanes highway, with randomized environment that contains other cars. The project needs to be able to drive an entire loop of 4.32 miles without conflicts like collisions, speed violation (the limit is 50mph) and acceleration (and deceleration) below 10m/s^2.

The main workflow has the following steps:

- Receive input data from the simulator.
- Sensor fusion data update and processing.
- Path planning.
- Trajectory generator.
- Send trajectory data to the simulator.

#### Sensor fusion processing

In this step, the project gets the sensor fucion data from the simulator and organizes it for future management. It basically processes the information about the other cars around the 'ego_car', calculating the corresponding lane, and saving them in a 3d vector called 'sensor_cars_lanes', where indexs are: '0->Left lane, 1->Center lane and 2->Right lane

In a second step, it checks if there's a close car in the same lane.

#### Path planning
The Path planning is based on a Finite State Machine with 3 states Keep Lane, Lane Change Left, Lane Change Right When needed (the front car is too close). After a list of valid states is returned, it calculates the associated cost for each one.

The cost functions used in this module are defined in `src/costs.h` and are basically two (a third one is defined, but not used in this version):

- Cost function for Change Lane state: This function, makes a double check. First of all, it checks if there's a car behind us in the destination lane, below a 'collision_distance' value. If there isn't a collision risk, if looks for the closest car in front of it, and calculates a cost depending on the distance. The furthest is the car, the lower is the cost.

- Cost function for Keep Lane state. It detects the closest car in front of the ego_car and it calculates the cost depending on the distance. Two buffer variables are defined to control the behavior:

 - `buffer_risk` is the 'security' distance the ego_car cannot overpass. If it does, then there's danger of collision and the cost is maximum.
 - `buffer_max` is the distance to start preventing the collision. In this case, the cost is calculated using a linear function. The closer 's' to 'buffer_max', the lower the cost, and viceversa.

Finally it assigns the new lane id and corrects the speed.

#### Trajectory generator

In order to drive the car along the highway, the Frenet coordinate transformations are used. Frenet space is a mathematical transformation where we say `d` is the center of the road, and `s` is how far down the road we are. To do this, we need a way to transform from (x,y) to (s, d), and viceversa. In `src/main.cpp` there are two functions to do this: `getFrenet(x,y,s)` and `getXY(s,d)`

This transformation makes it very easy to specify the car's lane and also the distance with other cars.

At this point, we know where the car is right now, and we know wether it should do a lane change. To create a smooth and nice trajectory, we use the `src/spline.h` library, creating a list of the current position and three widely spaced (x,y) waypoints, evenly spaced at 30m. Then we interpolate them with the `s.set_points(ptsx, ptsy)` call and the trajectory is defined.



### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

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
