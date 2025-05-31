
# -*- coding: utf-8 -*-
"""
Created on Thu May 01 21:39:27 2025

@author: Ammar Abdurrauf
"""

import uav
import time
import serial
import struct

from utils import switch_tab, set_camera_behind

# constants
X = -998

class Simulation:
    """
    This class is for the UAV take off and landing in X-Plane 11 flight simulator.
    """
    def __init__(self, arduino_port='COM7'):
        self.uav = uav.UAV()
        self.arduino = serial.Serial(port=arduino_port, baudrate=115200, timeout=.1)

        self.has_crashed = 0.0
        self.has_landed = False
        self.set_engine_and_flap_takeoff = True
        self.accumulated_pitch_error = 0
        self.cruise_i = 0
        self.phase = 'takeoff'
        self.initial_heading = self.uav.get_heading()

        self.state_history = []
        self.action_history = []

    def set_view(self):
        """
        Set the camera view to the UAV.
        """
        switch_tab()
        time.sleep(0.5)
        set_camera_behind()
        time.sleep(0.5)

    def simulate(self):
        self.set_view()

        while not self.has_crashed and not self.has_landed:
            state = self.uav.get_states()
            altitude = state[0]
            pos_x = state[1]
            pos_y = state[2]
            lat = state[3]
            long = state[4]
            speed_kias = state[5]
            ver_velocity = state[6]
            pitch = state[7]
            roll = state[8]
            yaw = state[9]
            pitch_rate = state[12]
            roll_rate = state[13]
            yaw_rate = state[14]
            self.has_crashed = state[15]

            # save the state history
            self.state_history.append([altitude, pos_x, pos_y, lat, long, speed_kias, ver_velocity,
                            pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate])
            
            print(f"Received flight data:"
                  f"\tAltitude: {altitude:.2f}"
                  f"\tAirspeed: {speed_kias:.2f}"
                  f"\tPitch:    {pitch:.2f}\n")

            # Pack 14 floats (4 bytes each) = 56 bytes
            data_struct = struct.pack('<14f',
                            altitude, pos_x, pos_y, lat, long, speed_kias, ver_velocity,
                            pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate,
                            self.initial_heading)
            self.arduino.write(data_struct)
            
            # read control data from arduino
            data = self.arduino.read(20)
            print("len(data):", len(data))
            if len(data) >= 20:
                elevator, aileron, rudder, throttle, flaps = struct.unpack('fffff', data)
                self.uav.send_control(elevator=elevator, aileron=aileron,
                                      rudder=rudder, throttle=throttle, flaps=flaps)
                print(f"Received control data:"
                      f"\tElevator: {elevator:.2f}"
                      f"\tAileron:  {aileron:.2f}"
                      f"\tRudder:   {rudder:.2f}"
                      f"\tThrottle: {throttle:.0f}"
                      f"\tFlaps:    {flaps:.0f}\n")
                
                # save the control history
                self.action_history.append([elevator, aileron, rudder, throttle, flaps])

            elif len(data) == 12 and round(sum(struct.unpack('fff', data)), 2) == 13.68:
                print("Landed signal received from flight computer\nThe UAV has landed successfully.")
                self.has_landed = True
            else:
                print("\ndata not received: ", data, "\n")


sim = Simulation()
sim.simulate()
