#ifndef ADD_SENSOR_INCLUDED
#define ADD_SENSOR_INCLUDED

using namespace std;

bool sensor_processing(vector<vector<double>> sensor_fusion,
  int prev_size,
  double car_s,
  int lane,
  map<int, vector<vector<double>>> &sensor_cars_lanes) {

  bool too_close = false;

  // using sensor fusion to know about each car in the road
  for (int i = 0; i < sensor_fusion.size(); i++) {
    // what lane the other car is in?
    double sensor_id = sensor_fusion[i][0];
    double sensor_vx = sensor_fusion[i][3];
    double sensor_vy = sensor_fusion[i][4];
    double sensor_s = sensor_fusion[i][5];
    double sensor_d = sensor_fusion[i][6];
    double sensor_speed = sqrt(sensor_vx * sensor_vx + sensor_vy * sensor_vy);

    sensor_s += ((double)prev_size * .02 * sensor_speed);  // if using previous points can project s value out

    vector<double> sensor_car = {sensor_id, sensor_s, sensor_d, sensor_speed};

    if (sensor_d > 8.0) {
      sensor_cars_lanes[2].push_back(sensor_car);
    } else if (sensor_d > 4.0) {
      sensor_cars_lanes[1].push_back(sensor_car);
    } else {
      sensor_cars_lanes[0].push_back(sensor_car);
    }

    // check if same lane
    if (sensor_d < (2 + 4 * lane + 2) && sensor_d > (2 + 4 * lane - 2)) {
      // check if s value is greater than ego_car and check the s gap is less than 30m
      if ((sensor_s > car_s) && ((sensor_s - car_s) < 30)) {
        too_close = true;
      }
    }
  }

  // hack to print a formatted list
  auto print_sensor_car_lanes = [](vector<vector<double>> sensor_cars_lanes, string title) {
    cout << title << endl;
    for (int i=0; i<sensor_cars_lanes.size(); i++) {
      for (auto j = sensor_cars_lanes[i].begin(); j != sensor_cars_lanes[i].end(); ++j)
        cout << *j << " ";
      cout << endl;
    }
  };

  /*
  system("clear");
  print_sensor_car_lanes(sensor_cars_lanes[0], "Sensor Info for Left Lane");
  print_sensor_car_lanes(sensor_cars_lanes[1], "Sensor Info for Mid Lane");
  print_sensor_car_lanes(sensor_cars_lanes[2], "Sensor Info for Right Lane");
  */
  return too_close;
}

#endif // ADD_SENSOR_INCLUDED
