#include <math.h>
#define DATA_SIZE 56

float values[14];
float target_altitude = 10.0; // target altitude
float descent_altitude = 6.0;
float flare_altitude = 1.4;
float touchdown_altitude = 0.15;

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

// clamp value between -1 and 1
float clamp(float value) {
  return fmax(fmin(value, 1.0f), -1.0f);
}

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

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);

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
    flight_data.longitude     = values[4];
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
      if (cruise_i < 20) {
        flight_data.use_pitch_stabilizer = 1;
        flight_data.use_roll_stabilizer = 1;
        flight_data.use_yaw_stabilizer = 1;
        flight_data.pitch_target = 0.0;
        ControlData ctrl = stabilize_flight(flight_data);
        flight_data.acc_pitch_err_fd = ctrl.acc_pitch_err_ctr;
        cruise_i++;

        control[0] = ctrl.elevator;
        control[1] = ctrl.aileron;
        control[2] = ctrl.rudder;
        control[3] = 0.4; // set throttle to 0.4 (cruise mode)
        control[4] = 0.0; // flaps to neutral in cruise
        Serial.write((byte*)control, sizeof(control));
      } else {
        phase++;
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