
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

        while not self.has_crashed:
            state = self.uav.get_states()
            altitude = state[0]
            pos_x = state[1]
            pos_y = state[2]
            speed_kias = state[3]
            ver_velocity = state[4]
            pitch = state[5]
            roll = state[6]
            yaw = state[7]
            pitch_rate = state[10]
            roll_rate = state[11]
            yaw_rate = state[12]
            self.has_crashed = state[13]

            # Pack 12 floats (4 bytes each) = 48 bytes
            data_struct = struct.pack('<12f',
                            altitude, pos_x, pos_y, speed_kias, ver_velocity,
                            pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate,
                            self.initial_heading)
            self.arduino.write(data_struct)
            
            # read control data from arduino
            data = self.arduino.read(20)
            if len(data) >= 20:
                elevator, aileron, rudder, throttle, flaps = struct.unpack('fffff', data)
                self.uav.send_control(elevator=elevator, aileron=aileron,
                                      rudder=rudder, throttle=throttle, flaps=flaps)
                print(f"Received control data:"
                      f"\n  Elevator: {elevator:.2f}"
                      f"\n  Aileron:  {aileron:.2f}"
                      f"\n  Rudder:   {rudder:.2f}"
                      f"\n  Throttle: {throttle:.0f}"
                      f"\n  Flaps:    {flaps:.0f}")
            else:
                print("\ndata not received: ", data, "\n")


sim = Simulation()
sim.simulate()
