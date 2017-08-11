#ifndef ADD_PLANNING_INCLUDED
#define ADD_PLANNING_INCLUDED

#include "costs.h"

using namespace std;


/*
  * The Path planning is based on a Finite State Machine with 3 states
  * Keep Lane, Lane Change Left, Lane Change Right
  * When needed (the front car is too close), this function finds
  * calculates the valid states where to move next, it calculates the
  * associated cost for each one, and finally changes the lane and the speed
  */
void path_planning(string state,
                   double car_s,
                   int &lane,
                   double &ref_vel,
                   map<int, vector<vector<double>>> sensor_cars_lanes) {

  // Find the valid states for the Finite State Machine
  vector<string> valid_states;
  if (state.compare("KL") == 0) {
    valid_states = {"KL", "LCL", "LCR"};
  } else if (state.compare("LCR") == 0) {
    valid_states = {"KL"};
  } else if (state.compare("LCL") == 0) {
    valid_states = {"KL"};
  }

  // We need to remove edge cases.
  // Cannot move left from left-most lane or
  // right from right-most lane.
  if (lane == 0) {
    valid_states.erase(std::remove(valid_states.begin(), valid_states.end(), "LCL"), valid_states.end());
  } else if (lane == 2) {
    valid_states.erase(std::remove(valid_states.begin(), valid_states.end(), "LCR"), valid_states.end());
  }

  double front_car_speed = 49.5;

  // Calculate the costs for each valid state
  map<string, double> costs;
  if (find(valid_states.begin(), valid_states.end(), "KL") != valid_states.end())
    costs["KL"]   = keep_lane_cost(car_s, sensor_cars_lanes[lane], front_car_speed); // + speed_cost(ref_vel);
  if (find(valid_states.begin(), valid_states.end(), "LCL") != valid_states.end())
    costs["LCL"]  = change_lane_cost(car_s, sensor_cars_lanes[lane - 1]);
  if (find(valid_states.begin(), valid_states.end(), "LCR") != valid_states.end())
    costs["LCR"]  = change_lane_cost(car_s, sensor_cars_lanes[lane + 1]);

  // Find the best next state going thru each valid
  // state and finding the one with min cost.
  double max_cost = 1E12;
  cout << "-------------------" << endl;
  for (auto test_state: valid_states) {
    double test_cost = costs[test_state];
    cout << " checking next state: " << test_state << ", cost: " << test_cost << endl;
    if (test_cost < max_cost) {
      max_cost = test_cost;
      state = test_state;
    }
  }
  cout << " --> NEXT state: " << state << endl;

  // With the new state defined, we can make adjustements to the speed or the lane
  if (state.compare("KL") == 0) {
    // If we decide to keep the same lane, then we need to adapt the speed
    // to the speed of the front car, to avoid collision.
    // TODO: find a better way to adapt the speed to the front car.
    if (fabs(front_car_speed - ref_vel) < .224) {
      ref_vel = front_car_speed;
    } else {
      if (front_car_speed > ref_vel) {
        ref_vel += .224;
      } else {
        ref_vel -= .224 / 4;
      }
    }
  } else if (state.compare("LCL") == 0) {
    lane -= 1;
  } else if (state.compare("LCR") == 0) {
    lane += 1;
  }
}

#endif // ADD_PLANNING_INCLUDED
