
# -*- coding: utf-8 -*-
"""
Created on Thu May 01 21:39:27 2025

@author: Ammar Abdurrauf
"""

import os
import uav
import time
import serial
import struct
import random
import math
import matplotlib.pyplot as plt
from datetime import datetime
from itertools import permutations

from utils import switch_tab, set_camera_behind


# the bounding coordinates (lat, lon)
A = [40.98651667882724, 29.21878590015432]
B = [40.98653153123276, 29.209383731215187]
C = [40.99910588578329, 29.210257488500844]
D = [40.99909130112695, 29.219542797172146]

# constants
X = -998
DATA_SIZE = 28

class Simulation:
    """
    This class is for the UAV take off and landing in X-Plane 11 flight simulator.
    """
    def __init__(self, arduino_port='COM7'):
        self.uav = uav.UAV()
        self.arduino = serial.Serial(port=arduino_port, baudrate=115200, timeout=.1)

        self.has_crashed = 0.0
        self.has_landed = False
        self.initial_heading = self.uav.get_heading()
        self.initial_pos = None

        self.uav.resume_sim()

        self.best_order = None
        self.uav_trajectory = []
        self.corners = [A, B, C, D]

    def plot_waypoints(self, initial_coordinate, corners, waypoint_order, uav_trajectory=None, show_plt=True):
        """
        Plots the bounding box, initial UAV position, waypoints, and the UAV's trajectory plan.
        If uav_trajectory is provided, it overlays the real UAV path on top of the plan.
        """
        # Extract coordinates
        corner_lats = [pt[0] for pt in corners] + [corners[0][0]]
        corner_lons = [pt[1] for pt in corners] + [corners[0][1]]

        waypoint_lats = [pt[0] for pt in waypoint_order]
        waypoint_lons = [pt[1] for pt in waypoint_order]

        init_lat, init_lon = initial_coordinate

        plt.figure(figsize=(8, 10))

        # Plot bounding box
        plt.plot(corner_lons, corner_lats, 'b--', label='Bounding Box')

        # Plot real UAV trajectory if provided
        if uav_trajectory is not None:
            uav_lats = [pt[0] for pt in uav_trajectory]
            uav_lons = [pt[1] for pt in uav_trajectory]
            plt.plot(uav_lons, uav_lats, color='orange', linewidth=2, label='UAV Real Trajectory')
            # plt.scatter(uav_lons, uav_lats, color='orange', s=5)  # optional: mark each point

        # Plot waypoints
        plt.scatter(waypoint_lons, waypoint_lats, color='red', label='Waypoints')
        for i, (lat, lon) in enumerate(zip(waypoint_lats, waypoint_lons), 1):
            plt.text(lon, lat, f'{i}', fontsize=9, ha='right')

        # Plot initial coordinate
        plt.scatter(init_lon, init_lat, marker='^', color='green', s=100, label='Initial Position')
        plt.text(init_lon, init_lat, 'Start', fontsize=10, ha='left')

        # Draw planned trajectory
        traj_points = [initial_coordinate] + waypoint_order
        for i in range(len(traj_points) - 1):
            lat1, lon1 = traj_points[i]
            lat2, lon2 = traj_points[i + 1]
            plt.annotate("",
                        xy=(lon2, lat2), xycoords='data',
                        xytext=(lon1, lat1), textcoords='data',
                        arrowprops=dict(arrowstyle="->", color='gray', lw=1.5))

        plt.xlabel("Longitude")
        plt.ylabel("Latitude")
        plt.title("UAV Waypoints and Planned Trajectory")
        plt.legend()

        plt.gca().invert_xaxis()
        plt.gca().invert_yaxis()
        plt.grid(True)
        plt.axis('equal')

        # Save the figure
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = f"D:/Projects/uav_arduino/pics/uav_trajectory_{timestamp}.png"
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=300)
        print(f"Saved trajectory plot to: {save_path}")

        if show_plt:
            plt.show()

    def set_view(self):
        """
        Set the camera view to the UAV.
        """
        switch_tab()
        time.sleep(0.5)
        set_camera_behind()
        time.sleep(0.5)

    def run(self):
        """
        This run the simulation in Python, not arduino.
        """
        self.set_view()

        self.has_crashed = False

        uav_phase = 1 # 1: take off - 2: mission - 3: landing

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

            current_pos = [lat, long, altitude]

            self.uav_trajectory.append(current_pos)


            # Pack 14 floats (4 bytes each) = 56 bytes
            data_struct = struct.pack('<14f',
                            altitude, pos_x, pos_y, lat, long, speed_kias, ver_velocity,
                            pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate,
                            self.initial_heading)
            
            self.arduino.write(data_struct)

            if uav_phase == 1:
                data = self.arduino.read(DATA_SIZE)
                if len(data) >= DATA_SIZE:
                    elevator, aileron, rudder, throttle, flaps, uav_phase, _ = struct.unpack('fffffff', data)
                    self.uav.send_control(elevator=elevator, aileron=aileron,
                                        rudder=rudder, throttle=throttle, flaps=flaps)
                    print("uav_phase 1:", uav_phase)
            elif uav_phase == 2:
                self.initial_pos = [lat, long]
                data = self.arduino.read(32)
                print("len(data):", len(data))
                
                if len(data) == 32:
                    lat1, long1, lat2, long2, lat3, long3, lat4, long4 = struct.unpack("ffffffff", data)
                    self.best_order = [[lat1, long1], [lat2, long2], [lat3, long3], [lat4, long4]]
                    self.plot_waypoints([lat, long],
                                        self.corners,
                                        self.best_order,
                                        show_plt=False)
                    print("uav_phase 2:", uav_phase)
                    uav_phase = 2.1
            elif round(uav_phase, 1) == 2.1:
                data = self.arduino.read(DATA_SIZE)
                if len(data) >= DATA_SIZE:
                    elevator, aileron, rudder, throttle, flaps, uav_phase, dist_to_target = struct.unpack('fffffff', data)
                    self.uav.send_control(elevator=elevator, aileron=aileron,
                                        rudder=rudder, throttle=throttle, flaps=flaps)
                    print("dist_to_target:", dist_to_target)
            elif uav_phase == 3.0:
                data = self.arduino.read(DATA_SIZE)
                if len(data) >= DATA_SIZE:
                    elevator, aileron, rudder, throttle, flaps, uav_phase, _ = struct.unpack('fffffff', data)
                    self.uav.send_control(elevator=elevator, aileron=aileron,
                                        rudder=rudder, throttle=throttle, flaps=flaps)
                    print("uav_phase 3:", uav_phase)
            else:
                print("Landing complete.")
                self.has_landed = True
        
        self.plot_waypoints(self.initial_pos, self.corners, self.best_order, uav_trajectory=self.uav_trajectory)


sim = Simulation()
corners = [A, B, C, D]
sim.run()
