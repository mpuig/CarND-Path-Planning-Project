#ifndef ADD_PLANNING_INCLUDED
#define ADD_PLANNING_INCLUDED

#include "costs.h"

using namespace std;

void plath_planning(string state,
                         double car_s,
                         int &lane,
                         double &ref_vel,
                         map<int, vector<vector<double>>> sensor_cars_lanes) {

  vector<string> valid_states;
  if (state.compare("KL") == 0) {
    valid_states = {"KL", "LCL", "LCR"};
  } else if (state.compare("LCR") == 0) {
    valid_states = {"KL"};
  } else if (state.compare("LCL") == 0) {
    valid_states = {"KL"};
  }

  // remove edge cases
  if (lane == 0) {
  // cannot move left from left-most lane
    valid_states.erase(std::remove(valid_states.begin(), valid_states.end(), "LCL"), valid_states.end());
  }
  if (lane == 2) {
    // cannot move right from right-most lane
    valid_states.erase(std::remove(valid_states.begin(), valid_states.end(), "LCR"), valid_states.end());
  }

  // store the potential costs
  map<string, double> costs;

  // calc the costs for each valid state
  // if (find(valid_states.begin(), valid_states.end(), "KL") != valid_states.end())
  costs["KL"]   = keep_lane_cost(car_s, sensor_cars_lanes[lane]); // + speed_cost(ref_vel);
  // if (find(valid_states.begin(), valid_states.end(), "LCL") != valid_states.end())
  costs["LCL"]  = change_lane_cost(car_s, sensor_cars_lanes[lane - 1]);
  // if (find(valid_states.begin(), valid_states.end(), "LCR") != valid_states.end())
  costs["LCR"]  = change_lane_cost(car_s, sensor_cars_lanes[lane + 1]);

  // find the state with min cost
  double max_cost = 1E12;

  // got thru each valid state and find the one with min cost
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

  // Here we have the next state.
  if (state.compare("KL") == 0) {
    // if we decide to keep the same lane, then we need to adapt the speed
    // to the speed of the front car, to avoid collision
    ref_vel -= .224;
  } else if (state.compare("LCL") == 0) {
    // if we change the lane, we can start adapting the speed to
    // the front car in the left lane
    //ref_vel += .224;
    lane -= 1;
  } else if (state.compare("LCR") == 0) {
    // if we change the lane, we can start adapting the speed to
    // the front car in the right lane
    //ref_vel += .224;
    lane += 1;
  }
}


#endif // ADD_PLANNING_INCLUDED
