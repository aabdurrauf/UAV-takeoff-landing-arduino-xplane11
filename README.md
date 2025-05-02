# UAV-Takeoff-Landing-Arduino-XPlane11
This project demonstrates an autonomous takeoff and landing system for a fixed-wing UAV in a simulated environment using X-Plane 11, an Arduino-based flight computer, and a Python interface. The UAV used is named Raphael UAV owned by ATA Team, and the entire flight mission — from takeoff to landing — is autonomously managed by a microcontroller running a custom algorithm.

## ✈️ Project Overview
1. Objective: To use a microcontroller (Arduino) as a flight computer to autonomously control a fixed-wing UAV in the X-Plane 11 simulator.
2. Simulation Target: The Raphael UAV model (or any similar fixed-wing UAV model). [ATA Team - Raphael V2 UAV 1.0.0](https://forums.x-plane.org/files/file/87594-ata-team-raphael-v2-uav/)
3. Control Flow: The flight computer onboard Arduino receives flight data, runs a state machine algorithm for autonomous takeoff and landing, and sends control inputs (elevator, aileron, rudder, throttle, and flaps) back to the simulator via Python.

## 📁 Project Structure
```
UAV-Takeoff-Landing-Arduino-XPlane11/
├── python/
│   ├── xpc/            # NASA X-PlaneConnect library (UDP interface)
│   ├── pyKey/          # Library to simulate keypress inputs in Python
│   ├── takeoff_landing_arduino.py
│   ├── mission_take_off_landing.py
│   ├── uav.py
│   └── utils.py
└── uav_xplane/
    └── uav_takeoff_landing.ino  # Arduino source code (.ino file)
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
- Open the uav_takeoff_landing.ino file in Arduino IDE and upload it to your Arduino board.
2. Start X-Plane 11
- Load the UAV model and start a simulation scenario.
3. Run Python Script

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
3. Cruise
4. Descent
5. Flare
6. Landing

## Credits
* X-Plane 11 - [Download X-plane 11](https://www.x-plane.com/product/desktop/)
* Raphael UAV Model – [ATA Team - Raphael V2 UAV 1.0.0](https://forums.x-plane.org/files/file/87594-ata-team-raphael-v2-uav/)
* NASA X-PlaneConnect – [XPlaneConnect](https://github.com/nasa/XPlaneConnect)
* pyKey Library – [pyKey](https://github.com/gauthsvenkat/pyKey)
