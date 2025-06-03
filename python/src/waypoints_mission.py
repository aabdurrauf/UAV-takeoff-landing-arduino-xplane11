
# -*- coding: utf-8 -*-
"""
Created on Thu May 01 21:39:27 2025

@author: Ammar Abdurrauf    
"""

import os
import uav
import time
import random
import math
import matplotlib.pyplot as plt
from datetime import datetime
from itertools import permutations

from utils import switch_tab, set_camera_behind


initial_coordinate = [40.994960968686094, 29.218932375695502]
# the bounding coordinates (lat, lon)
A = [40.98651667882724, 29.21878590015432]
B = [40.98653153123276, 29.209383731215187]
C = [40.99910588578329, 29.210257488500844]
D = [40.99909130112695, 29.219542797172146]

# constants
X = -998

class Simulation:
    """
    This class is for the UAV take off and landing in X-Plane 11 flight simulator.
    """
    def __init__(self, arduino_port='COM7'):
        self.uav = uav.UAV()
        # self.arduino = serial.Serial(port=arduino_port, baudrate=115200, timeout=.1)

        self.has_crashed = 0.0
        self.has_landed = False
        self.set_engine_and_flap_takeoff = True
        self.accumulated_pitch_error = 0
        self.cruise_i = 0
        self.phase = 'takeoff'
        self.initial_heading = self.uav.get_heading()
        self.initial_pos = None

        self.state_history = []
        self.action_history = []
        self.waypoints = []
        self.set_waypoints()

        self.uav.resume_sim()

        self.best_order = None
        self.uav_trajectory = []
        self.corners = [A, B, C, D]


    def calculate_best_order_and_display(self):
        self.best_order, _ = self.solve_tsp()
        self.plot_waypoints(self.initial_pos, self.corners, self.best_order)
        return self.best_order

    def set_waypoints(self):
        self.waypoints = self.generate_random_points_in_area([A, B, C, D])
        print("Random Points (min 25m apart):")
        for i, pt in enumerate(self.waypoints, 1):
            print(f"Point {i}: {pt}")

    def haversine_distance(self, lat1, lon1, lat2, lon2, diameter=6378145.0):
        rad = math.pi / 180  # degrees to radians
        dlat = (lat2 - lat1) * rad
        dlon = (lon2 - lon1) * rad
        a = math.sin(dlat / 2)**2 + math.cos(lat1 * rad) * math.cos(lat2 * rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return diameter * c
    
    def bearing_angle(self, lat1, lon1, lat2, lon2):
        """
        Calculates the bearing from (lat1, lon1) to (lat2, lon2) in degrees.
        """
        rad = math.pi / 180
        lat1, lat2 = lat1 * rad, lat2 * rad
        dlon = (lon2 - lon1) * rad

        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        initial_bearing = math.atan2(x, y)
        bearing_deg = (math.degrees(initial_bearing) + 360) % 360
        return bearing_deg
    
    def angular_diff(self, b1, b2):
        """
        Returns the smallest difference between two angles (in degrees).
        """
        return min(abs(b1 - b2), 360 - abs(b1 - b2))
    
    def calculate_heading_error(self, current_heading, target_heading):
        heading_error = (target_heading - current_heading + 360) % 360
        if heading_error > 180:
            heading_error -= 360  # Normalize to [-180, 180]
            
        return heading_error
    
    def generate_random_points_in_area(self, corners, num_points=4, min_dist=50, diameter=6378145.0):
        lats = [corner[0] for corner in corners]
        lons = [corner[1] for corner in corners]
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)

        points = []
        attempts = 0
        max_attempts = 1000

        while len(points) < num_points and attempts < max_attempts:
            rand_lat = random.uniform(min_lat, max_lat)
            rand_lon = random.uniform(min_lon, max_lon)

            too_close = False
            for p in points:
                dist = self.haversine_distance(rand_lat, rand_lon, p[0], p[1], diameter)
                if dist < min_dist:
                    too_close = True
                    break

            if not too_close:
                points.append([rand_lat, rand_lon])

            attempts += 1

        if len(points) < num_points:
            raise ValueError("Could not generate enough points satisfying the distance constraint.")

        return points
    
    def plot_waypoints(self, initial_coordinate, corners, waypoint_order, uav_trajectory=None):
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

        plt.show()

    def solve_tsp(self, bearing_tolerance=30):
        """
        Solves TSP considering that the first visiting point should be roughly in front
        of the UAV (within ±bearing_tolerance degrees from its initial heading).
        """
        best_order = None
        min_distance = float('inf')

        while best_order == None:

            for perm in permutations(self.waypoints):
                first_wp = perm[0]
                bearing_to_first = self.bearing_angle(
                    self.initial_pos[0], self.initial_pos[1],
                    first_wp[0], first_wp[1]
                )

                bearing_error = self.angular_diff(self.initial_heading, bearing_to_first)
                if bearing_error > bearing_tolerance:
                    continue  # Skip permutations that begin with a point behind the UAV

                # Compute total distance of the path
                total_dist = self.haversine_distance(
                    self.initial_pos[0], self.initial_pos[1],
                    first_wp[0], first_wp[1]
                )

                for i in range(len(perm) - 1):
                    p1, p2 = perm[i], perm[i+1]
                    total_dist += self.haversine_distance(p1[0], p1[1], p2[0], p2[1])

                if total_dist < min_distance:
                    min_distance = total_dist
                    best_order = perm

            if best_order is None:
                bearing_tolerance + 30

        return list(best_order), min_distance

    def set_view(self):
        """
        Set the camera view to the UAV.
        """
        switch_tab()
        time.sleep(0.5)
        set_camera_behind()
        time.sleep(0.5)

    def stabilize_flight(self, 
                        yaw, yaw_rate, 
                        roll, roll_rate,
                        pitch=None, pitch_rate=None, accumulated_pitch_error=0,
                        yaw_target=180, 
                        roll_target=0,
                        pitch_target=0,
                        Pr=0.1, Dr=0.01,
                        Pa=0.01, Da=0.001,
                        Pe=0.1, Ie=0, De=0.01):
        
        # yaw stabilizer
        yaw_err = yaw_target - yaw
        rudder_value = yaw_err * Pr - yaw_rate * Dr
        rudder_value = max(min(rudder_value, 1.0), -1.0)
        # send_control(client, rudder=rudder_value)
        
        # roll stabilizer
        roll_err = roll_target - roll
        aileron_value = roll_err * Pa - roll_rate * Da
        aileron_value = max(min(aileron_value, 1.0), -1.0)
        # send_control(client, aileron=aileron_value)
        
        elevator_value = -998
        if pitch != None and pitch_rate != None:
            # pitch stabilizer
            pitch_err = pitch_target - pitch
            accumulated_pitch_error += pitch_err
            elevator_value = pitch_err * Pe + accumulated_pitch_error * Ie - pitch_rate * De
            elevator_value = max(min(elevator_value, 1.0), -1.0)
            # send_control(client, elevator=elevator_value)
        
        self.uav.send_control( 
                    elevator=elevator_value,
                    aileron=aileron_value, 
                    rudder=rudder_value)
        
        # return rudder_value, aileron_value, elevator_value, accumulated_pitch_error

    def go_to_waypoint(self, 
                    yaw, yaw_rate,
                    roll, roll_rate,
                    pitch, pitch_rate,
                    current_pos, waypoint, ver_vel,
                    Dp=0.004, Di=0, Dd=0.0012, target_alti=10):
        
        target_heading = self.bearing_angle(current_pos[0], current_pos[1], waypoint[0], waypoint[1])
        
        # error_heading = target_heading - yaw
        error_heading = self.calculate_heading_error(yaw, target_heading)
        if error_heading > 180:
            error_heading -= 360  # Normalize to [-180, 180]
        roll_target = max(min(error_heading * 0.36, 30.0), -30.0)
        roll_error = roll_target - roll
        aileron_value = roll_error * Dp - roll_rate * Dd
        aileron_value = max(min(aileron_value, 1.0), -1.0)
        
        target_alti = target_alti
        current_alti = current_pos[2]
        alti_err = target_alti - current_alti
        # accumulated_alti_error += alti_err
        pitch_target = max(min(1.4*alti_err, 40.0), -40.0)
        pitch_error = pitch_target - pitch
        elevator_value = pitch_error * 0.045 - pitch_rate * 0.01 # + accumulated_alti_error * 0.0001
        elevator_value = max(min(elevator_value, 1.0), -1.0)
        
        rudder = 0.0
        self.uav.send_control(elevator=elevator_value, aileron=aileron_value, rudder=rudder)
        
    def run(self):
        """
        This run the simulation in Python, not arduino.
        """
        self.set_view()

        self.has_crashed = False
        target_altitude = 7

        phase = 'takeoff'
        set_engine_and_flap_takeoff = True

        best_route_calculated = False
        best_order = []
        
        wp_index = 0

        landing_phase = 'approach'
        flare_altitude = 1.5

        while not self.has_crashed and not self.has_landed:
            state = self.uav.get_states()
            altitude = state[0]
            # pos_x = state[1]
            # pos_y = state[2]
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

            if phase == 'takeoff':
                self.stabilize_flight(yaw, yaw_rate, roll, roll_rate, yaw_target=self.initial_heading)

                if set_engine_and_flap_takeoff:
                    self.uav.send_control(throttle=1.0, flaps=0.2)
                    set_engine_and_flap_takeoff = False

                if speed_kias >= 18.0 and speed_kias < 20.0:
                    self.uav.send_control(elevator=0.4)
                    
                if speed_kias > 20.0:
                    phase = 'climb'
                
            elif phase == 'climb':
                
                if altitude < target_altitude:
                    self.stabilize_flight( yaw, yaw_rate, roll, roll_rate, pitch, 
                                          pitch_rate, pitch_target=10, yaw_target=self.initial_heading)
                    
                else:
                    phase = 'cruise'
                    self.uav.send_control(throttle=0.4, flaps=0.0)
            
            elif phase == 'cruise':
                # start the mission

                if not best_route_calculated:
                    self.initial_pos = self.uav.get_current_lat_long()
                    self.uav.pause_sim()
                    best_order = self.calculate_best_order_and_display()
                    self.uav.resume_sim()
                    best_route_calculated = True

                dist_to_target = self.haversine_distance(current_pos[0], current_pos[1], 
                                                         best_order[wp_index][0], best_order[wp_index][1])

                self.go_to_waypoint(yaw=yaw, yaw_rate=yaw_rate, roll=roll, roll_rate=roll_rate,
                                    pitch=pitch, pitch_rate=pitch_rate, current_pos=current_pos,
                                    waypoint=best_order[wp_index], ver_vel=ver_velocity)

                print("dist_to_target: {:.2f} m".format(dist_to_target))
                if dist_to_target < 10:
                    wp_index += 1
                    if wp_index >= len(best_order):
                        print('MISSION COMPLETE... Preparing to land')
                        phase = 'landing'

            elif phase == 'landing':
                if landing_phase == 'approach':
                    self.uav.send_control(throttle=0.25, flaps=0.5)  # Reduce throttle, extend flaps
                    self.stabilize_flight(yaw, yaw_rate, roll, roll_rate, pitch, pitch_rate,
                                          roll_target=0, pitch_target=-5)
                    if altitude <= target_altitude:
                        landing_phase = 'final_descent'
                
                elif landing_phase == 'final_descent':
                    self.uav.send_control(throttle=0.1)  # Further reduce throttle
                    self.stabilize_flight(yaw, yaw_rate, roll, roll_rate, pitch, pitch_rate,
                                          yaw_target=yaw, roll_target=0, pitch_target=-7)
                    if altitude <= flare_altitude:
                        landing_phase = 'flare'
                
                elif landing_phase == 'flare':
                    self.uav.send_control(throttle=0.08)  # Cut throttle
                    self.stabilize_flight(yaw, yaw_rate, roll, roll_rate, pitch, pitch_rate,
                                          yaw_target=yaw, roll_target=0, pitch_target=2)  # Slightly nose up
                    if altitude < 0.5:  # Touchdown
                        landing_phase = 'complete'
                
                elif landing_phase == 'complete':
                    self.uav.send_control(throttle=0.0, flaps=0.0, gear=1)  # Deploy landing gear
                    print("Landing complete.")
                    self.has_landed = True
                
                print(f"PHASE: {landing_phase}, ALT: {altitude:.2f}, SPD: {speed_kias:.2f}, PITCH: {pitch:.2f}")
                time.sleep(0.1)
        
        self.plot_waypoints(self.initial_pos, self.corners, self.best_order, uav_trajectory=self.uav_trajectory)


sim = Simulation()
corners = [A, B, C, D]
sim.run()
