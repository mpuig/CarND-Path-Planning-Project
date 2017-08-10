#ifndef ADD_PLANNING_INCLUDED
#define ADD_PLANNING_INCLUDED

using namespace std;

int finite_state_machine(string state,
                         double car_s,
                         int lane,
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

  // Define Cost Functions as lambdas
  auto keep_lane_cost = [](double car_s, vector<vector<double>> sensor_cars_lane, double &ref_vel) {
    bool collision = false;
    // see if there are any collisions, if so, add a HUGE cost
    for (int i = 0; i < sensor_cars_lane.size(); i++) {
      double sensor_id = sensor_cars_lane[i][0];
      double sensor_s = sensor_cars_lane[i][1];
      if ((sensor_s > car_s) && ((sensor_s - car_s) < 30)) {
        collision = true;
      }
    }
    if (collision) {
      ref_vel -= .224;  // 5m/sec
      return 1000.0;
    } else {
      return 0.0;
    }
  };


  auto change_lane_cost = [](double car_s, vector<vector<double>> sensor_cars_lane) {
    bool collision = false;

    // see if there are any collisions, if so, add a HUGE cost
    // TODO: el cost hauria de disminuir si la distància del darrera és major
    for (int i = 0; i < sensor_cars_lane.size(); i++) {

      double sensor_id = sensor_cars_lane[i][0];
      double sensor_s = sensor_cars_lane[i][1];
      // check front cars
      if ((sensor_s > car_s) && ((sensor_s - car_s) < 30)) {
        collision = true;
      }
      // check back cars
      if ((sensor_s < car_s) && ((car_s - sensor_s) < 30)) {
        collision = true;
      }
    }
    if (collision) {
      cout << "collision" << endl;
      return 100000.0;
    } else {
      return 0.0;
    }
  };

  auto speed_cost = []() {
    /*
    auto cst =  0.5 * (target_speed - v);
    return cst * cst;
    */ return 0;
  };

  // store the potential costs
  map<string, double> costs;

  // calc the costs for each valid state
  if (find(valid_states.begin(), valid_states.end(), "KL") != valid_states.end())
    costs["KL"]   = keep_lane_cost(car_s, sensor_cars_lanes[lane], ref_vel) + speed_cost();
  if (find(valid_states.begin(), valid_states.end(), "LCL") != valid_states.end())
    costs["LCL"]  = change_lane_cost(car_s, sensor_cars_lanes[lane - 1]);
  if (find(valid_states.begin(), valid_states.end(), "LCR") != valid_states.end())
    costs["LCR"]  = change_lane_cost(car_s, sensor_cars_lanes[lane + 1]);

  // find the state with min cost
  double max_cost = 1E12;

  // got thru each valid state and find the one with min cost
  for (auto test_state: valid_states) {
      double test_cost = costs[test_state];

      cout << " checking next state: " << test_state << ", cost: " << test_cost << endl;
      if (test_cost < max_cost) {
          max_cost = test_cost;
          state = test_state;
          cout << " --> NEXT state: " << state << " <-- " << endl;
      }
  }
  cout << "cars left: " << sensor_cars_lanes[0].size() << endl;
  cout << "cars mid: " << sensor_cars_lanes[1].size() << endl;
  cout << "cars right: " << sensor_cars_lanes[2].size() << endl;

  if (state.compare("LCL") == 0) {
    return lane - 1;
  } else if (state.compare("LCR") == 0) {
    return lane + 1;
  }
  return lane;
}


#endif // ADD_PLANNING_INCLUDED
