#include <math.h>
#include <float.h>
#include "utils.h"

#define DATA_SIZE 56
#define MAX_POINTS 4
#define NUM_WAYPOINTS 4
#define MAX_ATTEMPTS 1000
#define EARTH_DIAMETER 6378145.0f
#define DEG_TO_RAD 0.017453292519943295f  // π / 180
#define RAD_TO_DEG 57.29577951308232f     // 180 / π
#define BEARING_TOLERANCE 30.0f

float values[14];
float target_altitude = 10.0; // target altitude
float descent_altitude = 6.0;
float flare_altitude = 1.4;
float touchdown_altitude = 0.15;
float initial_heading = -1.0;
int waypoints_generated = 0;
int waypoint_idx = 0;
float distance_to_wp = FLT_MAX;

// 0: take off, 1: climb, 2: cruise, 3: approach, 
// 4: final descent, 5: flare, 6: landing, 7: complete
unsigned char phase = 0; 
unsigned char set_engine_and_flap_takeoff = 1; // set engine and flap for take off
unsigned char land_complete = 0;
const int X = -998;

// currently the uav only takeoff then lnad directly
// so we need a cruise limit. we set the limit to 20 clock
unsigned char cruise_i = 0;

// PID constants
const float Pr = 0.1; // proportional rudder
const float Dr = 0.01; // derivative rudder
const float Pa = 0.01; // prop. aileron
const float Da = 0.001; // der. aileron
const float Pe = 0.16; // prop. elevator
const float De = 0.01; // der. elevator
const float Ie = 0.01; // integral elevator

struct FlightData {
  float altitude;
  float pos_x;
  float pos_z;
  float latitude;
  float longitude;
  float airspeed;
  float ver_vel;
  float pitch;
  float roll;
  float yaw;
  float pitch_rate;
  float roll_rate;
  float yaw_rate;

  float pitch_target;
  float roll_target;
  float yaw_target;

  char use_pitch_stabilizer;
  char use_roll_stabilizer;
  char use_yaw_stabilizer;

  float acc_pitch_err_fd;
};

// FlightData* flight_data = (FlightData*)malloc(sizeof(FlightData));
FlightData flight_data; // allocate statically (better for Arduino)

struct ControlData {
  float elevator;
  float aileron;
  float rudder;
  float acc_pitch_err_ctr;
};

LatLon initial_position = { -1.0, -1.0 };
LatLon waypoints[MAX_POINTS];
LatLon corners[4] = {
  {40.98651667882724f, 29.21878590015432f},
  {40.98653153123276f, 29.209383731215187f},
  {40.99910588578329f, 29.210257488500844f},
  {40.99909130112695f, 29.219542797172146f}
};
LatLon best_order[NUM_WAYPOINTS];

ControlData stabilize_flight(const FlightData& fd) {
  // ControlData* ctrl = (ControlData*)malloc(sizeof(ControlData));
  ControlData ctrl = {};  // zero-initialize all values

  // yaw stabilizer
  if (fd.use_yaw_stabilizer) {
    ctrl.rudder = clamp((fd.yaw_target - fd.yaw) * Pr - fd.yaw_rate * Dr);
  }
  // roll stabilizer
  if (fd.use_roll_stabilizer){
    ctrl.aileron = clamp(-fd.roll * Pa - fd.roll_rate * Da);
  }
  // pitch stabilizer
  if (fd.use_pitch_stabilizer) {
    ctrl.elevator = clamp(
      (fd.pitch_target - fd.pitch) * Pe + 
      fd.acc_pitch_err_fd * Ie - 
      fd.pitch_rate * De
    );
    ctrl.acc_pitch_err_ctr = fd.acc_pitch_err_fd + fd.pitch_target - fd.pitch;
  }

  return ctrl;
}

// Generate random points within defined corners
int generate_random_points_in_area(LatLon corners[], int num_corners, LatLon output[], int num_points, float min_dist) {
  float min_lat = corners[0].lat, max_lat = corners[0].lat;
  float min_lon = corners[0].lon, max_lon = corners[0].lon;

  for (int i = 1; i < num_corners; ++i) {
    if (corners[i].lat < min_lat) min_lat = corners[i].lat;
    if (corners[i].lat > max_lat) max_lat = corners[i].lat;
    if (corners[i].lon < min_lon) min_lon = corners[i].lon;
    if (corners[i].lon > max_lon) max_lon = corners[i].lon;
  }

  int count = 0;
  int attempts = 0;

  while (count < num_points && attempts < MAX_ATTEMPTS) {
    float rand_lat = random_float(min_lat, max_lat);
    float rand_lon = random_float(min_lon, max_lon);

    int too_close = 0;
    for (int j = 0; j < count; ++j) {
      float dist = haversine_distance(rand_lat, rand_lon, output[j].lat, output[j].lon, EARTH_DIAMETER);
      if (dist < min_dist) {
        too_close = 1;
        break;
      }
    }

    if (!too_close) {
      output[count].lat = rand_lat;
      output[count].lon = rand_lon;
      count++;
    }

    attempts++;
  }

  return (count == num_points);  // return 1 if successful, 0 otherwise
}

void solve_tsp(LatLon waypoints[NUM_WAYPOINTS], LatLon initial_pos, float initial_heading, LatLon best_order_out[NUM_WAYPOINTS]) {
  float min_distance = FLT_MAX;

  LatLon current_order[NUM_WAYPOINTS];
  memcpy(current_order, waypoints, sizeof(LatLon) * NUM_WAYPOINTS);

  int indices[NUM_WAYPOINTS] = {0, 1, 2, 3};

  do {
    LatLon perm[NUM_WAYPOINTS];
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
      perm[i] = waypoints[indices[i]];
    }

    float bearing_to_first = bearing_angle(initial_pos.lat, initial_pos.lon, perm[0].lat, perm[0].lon);
    float bearing_error = angular_diff(initial_heading, bearing_to_first);

    if (bearing_error > BEARING_TOLERANCE) {
      continue;
    }

    float total_dist = haversine_distance(initial_pos.lat, initial_pos.lon, perm[0].lat, perm[0].lon, EARTH_DIAMETER);
    for (int i = 0; i < NUM_WAYPOINTS - 1; i++) {
      total_dist += haversine_distance(perm[i].lat, perm[i].lon, perm[i+1].lat, perm[i+1].lon, EARTH_DIAMETER);
    }

    if (total_dist < min_distance) {
      min_distance = total_dist;
      memcpy(best_order_out, perm, sizeof(LatLon) * NUM_WAYPOINTS);
    }

  } while (next_permutation(indices, NUM_WAYPOINTS));
}

ControlData go_to_waypoint(const FlightData& fd, LatLon target_waypoint, float Dp, float Di, float Dd) {
    ControlData control;
    float target_heading = bearing_angle(fd.latitude, fd.longitude, target_waypoint.lat, target_waypoint.lon);
    float error_heading = calculate_heading_error(fd.yaw, target_heading);

    if (error_heading > 180.0f) {
        error_heading -= 360.0f;
    }

    float roll_target = error_heading * 0.36f;
    roll_target = fmax(fmin(roll_target, 30.0f), -30.0f);  // Clamp to [-30, 30]

    float roll_error = roll_target - fd.roll;
    float aileron_value = roll_error * Dp - fd.roll_rate * Dd;
    aileron_value = clamp(aileron_value);

    float alti_err = target_altitude - fd.altitude;
    float pitch_target = 1.4f * alti_err;
    pitch_target = fmax(fmin(pitch_target, 40.0f), -40.0f);  // Clamp to [-40, 40]

    float pitch_error = pitch_target - fd.pitch;
    float elevator_value = pitch_error * 0.045f - fd.pitch_rate * 0.01f;
    elevator_value = clamp(elevator_value);

    control.elevator = elevator_value;
    control.aileron = aileron_value;
    control.rudder = 0.0f;

    return control;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  randomSeed(analogRead(0));

  memset(&flight_data, 0, sizeof(FlightData));
}

void loop() {
  if (Serial.available() >= 56) {
    byte buffer[56];
    Serial.readBytes(buffer, 56);  // Read all 56 bytes

    // Convert bytes to float values
    memcpy(values, buffer, 56);  // Copy raw bytes into float array

    flight_data.altitude     = values[0];
    flight_data.pos_x        = values[1];
    flight_data.pos_z        = values[2];
    flight_data.latitude     = values[3];
    flight_data.longitude    = values[4];
    flight_data.airspeed     = values[5];
    flight_data.ver_vel      = values[6];
    flight_data.pitch        = values[7];
    flight_data.roll         = values[8];
    flight_data.yaw          = values[9];
    flight_data.pitch_rate   = values[10];
    flight_data.roll_rate    = values[11];
    flight_data.yaw_rate     = values[12];
    flight_data.yaw_target   = values[13];

    // elevator, aileorn, rudder, throttle, flaps
    float control[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

    if (phase == 0) { // take off
      flight_data.use_pitch_stabilizer = 0;
      flight_data.use_roll_stabilizer = 1;
      flight_data.use_yaw_stabilizer = 1;
      ControlData ctrl = stabilize_flight(flight_data);

      if (set_engine_and_flap_takeoff) {
        control[1] = ctrl.aileron;
        control[2] = ctrl.rudder;
        control[3] = 1.0; // set full throttle
        control[4] = 0.2; // flaps to 0.2 in take off
        Serial.write((byte*)control, sizeof(control));
        set_engine_and_flap_takeoff = 0;
      }

      if (flight_data.airspeed < 18.0) {
        control[1] = ctrl.aileron;
        control[2] = ctrl.rudder;
        control[3] = 1.0; // set full throttle
        control[4] = 0.2; // flaps to 0.2 in take off
        Serial.write((byte*)control, sizeof(control));
      }

      if (flight_data.airspeed < 20.0 && flight_data.airspeed >= 18.0) {
        control[0] = 0.4;
        control[1] = ctrl.aileron;
        control[2] = ctrl.rudder;
        control[3] = 1.0; // set full throttle
        control[4] = 0.2; // flaps to 0.2 in take off
        Serial.write((byte*)control, sizeof(control));
      }

      if (flight_data.airspeed > 20.0) {
        phase++; // change to climb phase
      }

    } 
    else if (phase == 1) { // climb
      flight_data.use_pitch_stabilizer = 1;
      flight_data.use_roll_stabilizer = 1;
      flight_data.use_yaw_stabilizer = 1;
      flight_data.pitch_target = 10.0;
      ControlData ctrl = stabilize_flight(flight_data);

      if (flight_data.altitude < target_altitude) {
        control[0] = ctrl.elevator;
        control[1] = ctrl.aileron;
        control[2] = ctrl.rudder;
        control[3] = 1.0; // set full throttle
        control[4] = 0.2;
        Serial.write((byte*)control, sizeof(control));
      }
      else {
        phase++; // change to cruise phase
        control[0] = ctrl.elevator;
        control[1] = ctrl.aileron;
        control[2] = ctrl.rudder;
        control[3] = 0.4; // set throttle to 0.4 (cruise mode)
        control[4] = 0.0; // flaps to neutral in cruise
        Serial.write((byte*)control, sizeof(control));
      }
    }
    else if (phase == 2) { // cruise
      if (generate_random_points_in_area(corners, 4, waypoints, MAX_POINTS, 50.0f) && waypoints_generated == 0) {
        // solve the TSP problem
        if (initial_heading == -1.0) {
          initial_heading = flight_data.yaw;
          initial_position.lat = flight_data.latitude;
          initial_position.lon = flight_data.longitude;
        }
        solve_tsp(waypoints, initial_position, initial_heading, best_order);
        waypoints_generated = 1;
      } else {
        Serial.println("Failed to generate enough valid waypoints.");
      }

      // start the mission
      distance_to_wp = haversine_distance(flight_data.latitude, flight_data.longitude, best_order[waypoint_idx].lat, best_order[waypoint_idx].lon, EARTH_DIAMETER);
      ControlData ctrl = go_to_waypoint(flight_data, best_order[waypoint_idx], 0.004, 0.0, 0.0012);

      control[0] = ctrl.elevator;
      control[1] = ctrl.aileron;
      control[2] = ctrl.rudder;
      control[3] = 0.4; // set throttle to 0.4 (cruise mode)
      control[4] = 0.0; // flaps to neutral in cruise
      Serial.write((byte*)control, sizeof(control));

      if (distance_to_wp < 10) {
        waypoint_idx++;
        if (waypoint_idx >= 4) {
          phase++;
        }
      }
    }
    else if (phase == 3) { // descent / approaching target descent
      flight_data.use_pitch_stabilizer = 1;
      flight_data.use_roll_stabilizer = 1;
      flight_data.use_yaw_stabilizer = 1;
      flight_data.pitch_target = -5.0;
      ControlData ctrl = stabilize_flight(flight_data);

      control[0] = ctrl.elevator;
      control[1] = ctrl.aileron;
      control[2] = ctrl.rudder;
      control[3] = 0.22; // slowing down
      control[4] = 0.5;
      Serial.write((byte*)control, sizeof(control));

      if (flight_data.altitude <= descent_altitude) {
        phase++;
      }
    }
    else if (phase == 4) { // final descent
      flight_data.use_pitch_stabilizer = 1;
      flight_data.use_roll_stabilizer = 1;
      flight_data.use_yaw_stabilizer = 1;
      flight_data.pitch_target = -7.0;
      ControlData ctrl = stabilize_flight(flight_data);

      control[0] = ctrl.elevator;
      control[1] = ctrl.aileron;
      control[2] = ctrl.rudder;
      control[3] = 0.1; // slowing down
      control[4] = 0.5;
      Serial.write((byte*)control, sizeof(control));

      if (flight_data.altitude <= flare_altitude) {
        phase++;
      }
    }
    else if (phase == 5) { // flare
      flight_data.use_pitch_stabilizer = 1;
      flight_data.use_roll_stabilizer = 1;
      flight_data.use_yaw_stabilizer = 1;
      flight_data.pitch_target = -2.0;
      ControlData ctrl = stabilize_flight(flight_data);

      control[0] = ctrl.elevator;
      control[1] = ctrl.aileron;
      control[2] = ctrl.rudder;
      control[3] = 0.075; // slowing down
      control[4] = 0.5;
      Serial.write((byte*)control, sizeof(control));
      
      if (flight_data.altitude <= touchdown_altitude) {
        phase++;
      }
    }
    else if (phase == 6) { // landing
      control[0] = 0.0;
      control[1] = 0.0;
      control[2] = 0.0;
      control[3] = 0.0; 
      control[4] = 0.0; // shutdown all machine and neutral
      Serial.write((byte*)control, sizeof(control));
      if (flight_data.airspeed <= 0.1) {
        phase++;
      }
    }
    else {
      // send signal indicating the uav has landed
      float landed_signal[3] = {1.23, 4.56, 7.89};
      Serial.write((byte*)landed_signal, sizeof(landed_signal));
    }
  }
  else {
    // there are some missing data
  }
}