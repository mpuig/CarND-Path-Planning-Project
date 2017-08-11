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

// Define Cost Functions as lambdas
double keep_lane_cost(double car_s, vector<vector<double>> sensor_cars_lane, double &front_car_speed) {
  double cost = 0.0;
  double buffer_risk = 15.0;
  double buffer_max = 30.0;

  // if there's a car in front of the ego car, calculate
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


double change_lane_cost(double car_s, vector<vector<double>> sensor_cars_lane) {
  double cost = 0.0;

  // if there's a close car behind the ego car, calculate cost
  double collision_dist = 5.0;
  int back_id = get_back_car_in_lane(car_s, sensor_cars_lane);
  if (back_id > -1) {
    double sensor_s = sensor_cars_lane[back_id][1];
    double dist_s = car_s - sensor_s;
    if (dist_s <= collision_dist) {
      cost = 1.0;  // assign the maximum value to cost
    }
  }
  // if there isn't collision risk, find the closest car
  // in front of the ego car and calculate the cost.
  // The furthest is the car, the lower is the cost.
  if (cost < 1.0) {
    double buffer_risk = 5.0;
    double buffer_max = 100.0;
    int front_id = get_front_car_in_lane(car_s, sensor_cars_lane);
    if (front_id > -1) {
      double sensor_s = sensor_cars_lane[front_id][1];
      double dist_s = sensor_s - car_s;
      if (dist_s < buffer_risk) {
        cost = 1.0; // collision
      } else if ((dist_s >= buffer_risk) && (dist_s < buffer_max)) {
        cost = (buffer_max - dist_s) / (buffer_max - buffer_risk);
      }
    }
  }

  return cost;
};

double speed_cost(double ref_vel) {
  double stop_cost = 0.8;
  double speed_limit = 49.5;
  double buffer_v = 5.0;
  double target_speed = speed_limit - buffer_v;
  double cost = 1.0;
  if (ref_vel < target_speed) {
    cost = stop_cost * ((target_speed - ref_vel) / target_speed);
  } else if ((ref_vel > target_speed) && (ref_vel < speed_limit)) {
    cost = (ref_vel - target_speed) / buffer_v;
  }
  return cost;
};

#endif // ADD_COSTS_INCLUDED