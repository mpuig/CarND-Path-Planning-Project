#ifndef ADD_COSTS_INCLUDED
#define ADD_COSTS_INCLUDED

using namespace std;


int get_front_car_in_lane(double car_s, vector<vector<double>> sensor_cars_lane) {
  // find the closest car in front of the ego car
  double front_dist = 1E12;
  int front_id = -1;

  for (int i = 0; i < sensor_cars_lane.size(); i++) {
    double sensor_s = sensor_cars_lane[i][1];
    double dist_s = sensor_s - car_s;
    if ((dist_s > 0) && (dist_s < front_dist)) {
      front_dist = dist_s;
      front_id = i;
    }
  }
  return front_id;
}

int get_back_car_in_lane(double car_s, vector<vector<double>> sensor_cars_lane) {
  // find the closest car back of the ego car
  double back_dist = 1E12;
  int back_id = -1;

  for (int i = 0; i < sensor_cars_lane.size(); i++) {
    double sensor_s = sensor_cars_lane[i][1];
    double dist_s = car_s - sensor_s;
    if ((dist_s > 0) && (dist_s < back_dist)) {
      back_dist = dist_s;
      back_id = i;
    }
  }
  return back_id;
}

/*
 * Cost function for Keep Lane state.
 * It detects the closest car in front of the ego_car and it calculates
 * the cost depending on the distance. Two buffer variables are defined to
 * control the behavior:
 *
 * - buffer_risk is the 'security' distance the ego_car cannot overpass. If it does,
 * then there's danger of collision and the cost is maximum.
 *
 * - buffer_max is the distance to start preventing the collision. In this case, the cost
 * is calculated using a linear function. The closer 's' to 'buffer_max', the lower the cost, and viceversa.
 */
double keep_lane_cost(double car_s, vector<vector<double>> sensor_cars_lane, double &front_car_speed) {
  double cost = 0.0;

  // control variables
  double buffer_risk = 15.0;
  double buffer_max = 30.0;

  // is there a car in front of ego_car?
  int front_id = get_front_car_in_lane(car_s, sensor_cars_lane);

  if (front_id > -1) {
    double sensor_s = sensor_cars_lane[front_id][1];
    front_car_speed = sensor_cars_lane[front_id][3];
    double dist_s = sensor_s - car_s;
    if (dist_s < buffer_risk) {
      cost = 1.0; // collision
    } else if (dist_s < buffer_max) {
      cost = (buffer_max - dist_s) / (buffer_max - buffer_risk);
    }
  }
  return cost;
};

/*
 * Cost function for Change Lane state.
 * This function, makes a double check. First of all, it checks if there's a car behind us
 * in the destination lane, below a 'collision_distance' value.
 * If there isn't a collision risk, if looks for the closest car in front of it, and
 * calculates a cost depending on the distance. The furthest is the car, the lower is the cost.
 */
double change_lane_cost(double car_s, vector<vector<double>> sensor_cars_lane) {
  double cost = 0.0;

  // control variables
  double collision_dist = 5.0;

  // if there's a close car behind the ego car^
  int back_id = get_back_car_in_lane(car_s, sensor_cars_lane);

  if (back_id > -1) {
    double sensor_s = sensor_cars_lane[back_id][1];
    double dist_s = car_s - sensor_s;
    if (dist_s <= collision_dist) {
      cost = 1.0;  // assign the maximum value to cost
    }
  }
  // No car below the ego_car
  if (cost < 1.0) {
    double buffer_risk = 5.0;
    double buffer_max = 100.0;
    int front_id = get_front_car_in_lane(car_s, sensor_cars_lane);
    if (front_id > -1) {
      double sensor_s = sensor_cars_lane[front_id][1];
      double dist_s = sensor_s - car_s;
      if (dist_s < buffer_risk) {
        cost = 1.0;  // collision
      } else if ((dist_s >= buffer_risk) && (dist_s < buffer_max)) {
        cost = (buffer_max - dist_s) / (buffer_max - buffer_risk);
      }
    }
  }
  return cost;
};

/*
 * This functions calculates the cost associated to the speed.
 * the lowest the speed, the highest the cost.
 * It's also high if the speed is over the legal limits.
 */
double speed_cost(double ref_vel) {
  double cost = 1.0;

  // control variables
  double stop_cost = 0.8;
  double speed_limit = 49.5;
  double buffer_v = 5.0;
  double target_speed = speed_limit - buffer_v;

  if (ref_vel < target_speed) {
    cost = stop_cost * ((target_speed - ref_vel) / target_speed);
  } else if ((ref_vel > target_speed) && (ref_vel < speed_limit)) {
    cost = (ref_vel - target_speed) / buffer_v;
  }
  return cost;
};

#endif // ADD_COSTS_INCLUDED