# UAV Waypoints Arduino X-Plane 11
This project demonstrates an autonomous takeoff and landing system for a fixed-wing UAV in a simulated environment using X-Plane 11, an Arduino-based flight computer, and a Python interface. The UAV used is named Raphael UAV owned by the ATA Team, and the entire flight mission — from takeoff to landing — is autonomously managed by a microcontroller running a custom algorithm.

Additionally, this project implements an autonomous waypoint mission capability, where waypoint calculation and mission planning can be performed directly on the Arduino Uno, enabling fully onboard navigation control without reliance on external computation.

## ✈️ Project Overview
1. Objective: To use a microcontroller (Arduino) as a flight computer to autonomously control a fixed-wing UAV in the X-Plane 11 simulator, including waypoint navigation.
2. Simulation Target: The Raphael UAV model (or any similar fixed-wing UAV model). [ATA Team - Raphael V2 UAV 1.0.0](https://forums.x-plane.org/files/file/87594-ata-team-raphael-v2-uav/)
3. Control Flow: The flight computer onboard Arduino receives flight data, runs a state machine algorithm for autonomous takeoff, waypoint navigation, and landing, and sends control inputs (elevator, aileron, rudder, throttle, and flaps) back to the simulator via Python.

## 📁 Project Structure
```
UAV-Takeoff-Landing-Arduino-XPlane11/
├── python/
│   ├── xpc/              # NASA X-PlaneConnect library (UDP interface)
│   ├── pyKey/            # Library to simulate keypress inputs in Python
│   ├── takeoff_landing_arduino.py
│   ├── mission_take_off_landing.py
│   ├── waypoints_mission_arduino.py
│   ├── waypoints_mission.py
│   ├── waypoints_mission_land.py    # this is not done, if you want to finish this program code, just fork and work on it
│   ├── shift_aircraft.py            # for observation
│   ├── uav.py            # uav class (helper class)
│   └── utils.py
├── uav_xplane/
│   └── uav_takeoff_landing.ino      # Arduino source code (.ino file)
└── uav_waypoints/main    # for autonomous waypoint mission
    ├── main.ino
    ├── utils.cpp
    └── utils.h
```

- python/: Contains Python code to communicate with both X-Plane and Arduino.
  - Uses:
    - pyKey for keypress simulation
    - X-PlaneConnect (by NASA) for UDP communication with X-Plane 11
- uav_xplane/: Contains the Arduino .ino code that runs the UAV takeoff and landing algorithm.

## 🧰 Requirements
- X-Plane 11 simulator
- Arduino board + Arduino IDE
- Raphael UAV model (or similar fixed-wing aircraft)
- Python 3 with:
  - pyserial
  - xpc (X-PlaneConnect)
  - pyKey
  
## 🚀 How to Run
1. Flash Arduino Code
- Open the uav_takeoff_landing.ino (for take off and landing only) or main.ino (for autonomous waypoint mission) file in Arduino IDE and upload it to your Arduino board.
2. Start X-Plane 11
- Load the UAV model and start a simulation scenario.
3. Run Python Script according to the mission (can be inferred from the file name)

## 🔁 Data Flow
```
X-Plane 11
   ↓ (UDP via xpc)
Python (data processing & serial communication)
   ↓ (Serial via USB)
Arduino (runs autonomous control logic)
   ↓ (control outputs)
Python
   ↓ (UDP via xpc)
X-Plane 11

```

- Flight Data (altitude, airspeed, pitch, yaw, etc.) → from X-Plane to Arduino
- Control Inputs (throttle, rudder, elevator, aileron, flaps) → from Arduino to X-Plane

📌 Notes
The system is modular: you can replace the UAV model as long as it's a fixed-wing type.
The current control strategy is a phase-based state machine, covering:
1. Takeoff
2. Climb
3. Cruise (waypoint mission if used)
4. Descent
5. Flare
6. Landing

## Credits
* X-Plane 11 - [Download X-plane 11](https://www.x-plane.com/product/desktop/)
* Raphael UAV Model – [ATA Team - Raphael V2 UAV 1.0.0](https://forums.x-plane.org/files/file/87594-ata-team-raphael-v2-uav/)
* NASA X-PlaneConnect – [XPlaneConnect](https://github.com/nasa/XPlaneConnect)
* pyKey Library – [pyKey](https://github.com/gauthsvenkat/pyKey)
