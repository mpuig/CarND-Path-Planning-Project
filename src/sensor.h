#ifndef ADD_SENSOR_INCLUDED
#define ADD_SENSOR_INCLUDED

using namespace std;


/*
 * This function processes the information about the other cars around the 'ego_car'
 *
 * In a first step, it calculates the lane of each car.
 * The 'sensor_d' indicates the distance from the highway center.
 * Lanes are 4m width, so the Left lane is (0,4), the center one is (4,8)
 * and the right one is (8,12). Comparing the sensor_d value with the limits,
 * we can assign the correct lane
 *
 * 'sensor_cars_lanes' is a vector of lanes information, which indexs are:
 * Left lane-> 0, Center lane-> 1, Right lane -> 2
 *
 * In a second step, it checks if there's a close car in the same lane
 */
bool sensor_processing(vector<vector<double>> sensor_fusion,
  int prev_size,
  double car_s,
  int lane,
  map<int, vector<vector<double>>> &sensor_cars_lanes) {

  bool too_close = false;

  // using sensor fusion to know about each car in the road
  for (int i = 0; i < sensor_fusion.size(); i++) {
    double sensor_id = sensor_fusion[i][0];
    double sensor_vx = sensor_fusion[i][3];
    double sensor_vy = sensor_fusion[i][4];
    double sensor_s = sensor_fusion[i][5];
    double sensor_d = sensor_fusion[i][6];
    double sensor_speed = sqrt(sensor_vx * sensor_vx + sensor_vy * sensor_vy);

    // Using previous points we can project the s value out
    sensor_s += ((double)prev_size * .02 * sensor_speed);

    // Vector representation of a car.
    vector<double> sensor_car = {sensor_id, sensor_s, sensor_d, sensor_speed};

    if (sensor_d > 8.0) {
      sensor_cars_lanes[2].push_back(sensor_car);
    } else if (sensor_d > 4.0) {
      sensor_cars_lanes[1].push_back(sensor_car);
    } else {
      sensor_cars_lanes[0].push_back(sensor_car);
    }

    // Check if the car is in the same lane than ego car
    if (sensor_d < (2 + 4 * lane + 2) && sensor_d > (2 + 4 * lane - 2)) {
      // Check if 's' value is greater than ego_car and check the 's' gap is less than 30m
      if ((sensor_s > car_s) && ((sensor_s - car_s) < 30)) {
        too_close = true;
      }
    }
  }

  /*
  // Hack to print a formatted list
  auto print_sensor_car_lanes = [](vector<vector<double>> sensor_cars_lanes, string title) {
    cout << title << endl;
    for (int i=0; i<sensor_cars_lanes.size(); i++) {
      for (auto j = sensor_cars_lanes[i].begin(); j != sensor_cars_lanes[i].end(); ++j)
        cout << *j << " ";
      cout << endl;
    }
  };


  system("clear");  // this works only on OSX systems

  print_sensor_car_lanes(sensor_cars_lanes[0], "Sensor Info for Left Lane");
  print_sensor_car_lanes(sensor_cars_lanes[1], "Sensor Info for Mid Lane");
  print_sensor_car_lanes(sensor_cars_lanes[2], "Sensor Info for Right Lane");
  */

  return too_close;
}

#endif // ADD_SENSOR_INCLUDED
