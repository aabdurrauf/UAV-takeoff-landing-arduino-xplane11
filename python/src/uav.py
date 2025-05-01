# -*- coding: utf-8 -*-
"""
Created on Sun Apr 06 14:46:02 2025

@author: Ammar Abdurrauf
"""

import xpc
import time
import numpy as np

class UAV:
    """
    This class is for controlling the player UAV in X-Plane 11 flight simulator.
    """
    def __init__(self, xplane_client=None):
        if xplane_client is None:
            self.client = xpc.XPlaneConnect()
            # Verify connection
            try:
                # If X-Plane does not respond to the request, a timeout error will be raised.
                self.client.getDREF("sim/test/test_float")
            except:
                print("Error establishing connection to X-Plane.")
                self.client.close()
                self.client = None
                return
            print("Connection established-xplane")
        else:
            self.client = xplane_client

    def set_altitude(self, alt):
        X = -998  # set value to -998 to keep unchanged
        values = [X, X, alt, X, X, X, X]
        try:
            self.xplane.sendPOSI(values)
        except:
            self.client.clearBuffer()
            print(f"Error setting the altitude.")

    def shift_coordinate(self, x=None, y=None, z=None):
        if type(x) == int or type(x) == float:
            self.set_coordinate(x=self.xplane.getDREF(
                'sim/flightmodel/position/local_x')[0] + x)
        
        if type(y) == int or type(y) == float:
            self.set_coordinate(y=self.xplane.getDREF(
                'sim/flightmodel/position/local_y')[0] + y)
                
        if type(z) == int or type(z) == float:
            self.set_coordinate(z=self.xplane.getDREF(
                'sim/flightmodel/position/local_z')[0] + z)
            
    def set_coordinate(self, x=None, y=None, z=None):
        if type(x) == int or type(x) == float:
            try:
                self.xplane.sendDREF('sim/flightmodel/position/local_x', x)
            except:
                self.client.clearBuffer()
                print("Error setting the x coordinate.")
        
        if type(y) == int or type(y) == float:
            try:
                self.xplane.sendDREF('sim/flightmodel/position/local_y', y)
            except:
                self.client.clearBuffer()
                print("Error setting the y coordinate.") 
                
        if type(z) == int or type(z) == float:
            try:
                self.xplane.sendDREF('sim/flightmodel/position/local_z', z)
            except:
                self.client.clearBuffer()
                print("Error setting the z coordinate.")      

    def set_position(self, latitude=-998, longitude=-998, altitude=-998, 
                     pitch=-998, roll=-998, true_head=-998, gear=-998):
        
        if true_head != -998:
            ovr = [0] * 20
            ovr[0] = 1
            self.xplane.sendDREF('sim/operation/override/override_planepath', ovr)
            
        values = [latitude, longitude, altitude, pitch, roll, true_head, gear]
        try:
            self.xplane.sendPOSI(values)
        except:
            self.client.clearBuffer()
            print("Error setting the position.")
        
        time.sleep(0.5)
        if true_head != -998:
            ovr = [0] * 20
            self.xplane.sendDREF('sim/operation/override/override_planepath', ovr)
        
    def send_control(self, control_values = [], 
                     elevator=-998, aileron=-998, rudder=-998, 
                     throttle=-998, gear=-998, flaps=-998):
        
        if len(control_values) == 0:
            controls = [elevator, aileron, rudder, throttle, gear, flaps]
        else:
            controls = control_values            
            
        try:
            self.client.sendCTRL(controls)
        except:
            self.client.clearBuffer()
            print("Error sending controls.")

    def get_states(self):
        
        """
        Generally:

            sim/cockpit2 are in-cockpit indications - they will show the wrong 
            values when there is a systems failure. For example, the airspeed 
            in sim/cockpit2 will go to zero as the pitot tube ices up.

            sim/flightmodel2 are the real physics-simulation values - 
            they are correct always, and thus are good for modeling the 
            external plane. For example, the AoA in sim/flightmodel2 will 
            always show the real AoA no matter what electrical and vacuum 
            systems fail - this would be appropriate for the AoA sensor 
            mechanism on the outside of the plane.
            
            both sim/cockpit2 and sim/flightmodel2 work correctly for any 
            plane (the user’s or multi-player). This means the sim/flightmodel2 
            datarefs will show the values for the AI/multiplayer planes when 
            read from an object attached to an ACF loaded as an 
            AI/multiplayer plane.

            The older sim/cockpit and sim/flightmodel datarefs are not 
            divided into cockpit (with failures) vs. physics (no failures), 
            and always show the user’s plane, even when your OBJ is attached 
            to another plane. So it is best to use sim/cockpit2 for generic 
            instruments and sim/ flightmodel2/ everywhere else.
            
            https://developer.x-plane.com/manuals/planemaker/index.html#workingwiththeaircraftssystems
        """
        
        drefs = ['sim/flightmodel2/position/y_agl', # altitude
                 'sim/flightmodel/position/local_x', # positi on x
                 'sim/flightmodel/position/local_z', # position z (position y is the altitude)
                 'sim/flightmodel/position/indicated_airspeed', # Knots Indicated Airspeed (KIAS)
                 'sim/flightmodel/position/vh_ind', # vertical velocity
                 'sim/flightmodel/position/theta', # pitch angle
                 'sim/flightmodel/position/phi', # roll angle
                 'sim/flightmodel/position/psi', # yaw angle
                 # 'sim/flightmodel/forces/vx_acf_axis', # velocity across x axis
                 # 'sim/flightmodel/forces/vz_acf_axis', # velocity acros z axis
                 'sim/flightmodel/position/local_vx', # velocity across x axis
                 'sim/flightmodel/position/local_vz', # velocity acros z axis
                 'sim/flightmodel/position/Q', # pitch rate
                 'sim/flightmodel/position/P', # roll rate
                 'sim/flightmodel/position/R', # yaw rate
                 'sim/flightmodel2/misc/has_crashed'] # crash or not
        try:
            self.client.clearBuffer()
            values = self.client.getDREFs(drefs)
            values = np.array(values).flatten()            
            
            if len(values) < len(drefs):
                raise Exception()

            values[0] = self.ft_to_meter(values[0]) # convert feet to meter
        except:
            self.client.clearBuffer()
            # set values to 0 if error occurred in communication with x-plane
            values = np.zeros(13)
            print("Error getting the states.")
            
        return values

    def set_pitch_rate(self, angle):
        try:
            self.client.sendDREF('sim/flightmodel/position/Q', angle)
        except:
            print("Error setting the pitch rate.")
        
    def set_roll_rate(self, angle):
        try:
            self.client.sendDREF('sim/flightmodel/position/P', angle)
        except:
            print("Error setting the roll rate.")

    def wait_craft_to_reset(self):
        self.client.clearBuffer()
        crash = self.has_crashed()
        while crash == 1.0:
            self.client.clearBuffer()
            crash = self.has_crashed()
    
    def wait_craft_to_touch_ground(self):
        self.client.clearBuffer()
        on_ground = self.is_on_ground()
        while on_ground == 0.0:
            self.client.clearBuffer()
            on_ground = self.is_on_ground()
    
    def refuel_uav(self, fuel=10000):
        try:
            self.client.sendDREF('sim/flightmodel/weight/m_fuel1', fuel)
        except:
            print("Error refueling the UAV.")
    
    def get_fuel_cost(self, initial_fuel=294835.13):
        try:
            fuel_cost = (initial_fuel-self.client.getDREF('sim/flightmodel/weight/m_fuel1')[0])/initial_fuel
        except:
            print("Error getting the fuel cost.")
            fuel_cost = -1.0
        
        return fuel_cost
    
    def get_heading(self):
        return self.client.getDREF('sim/flightmodel/position/psi')[0]
    
    def sendDREF(self, dref, value):
        try:
            self.client.sendDREF(dref, value)
        except Exception as ex:
            print('error:', ex)
            
    def set_view_spot(self):
        # self.client.sendVIEW(xpc.ViewType.Spot)
        self.client.sendVIEW(xpc.ViewType.Follow)
    
    def pause_sim(self):
        self.client.pauseSim(True)
    
    def resume_sim(self):
        self.client.pauseSim(False)
    
    def ft_to_meter(self, ft):
        return ft*0.3048