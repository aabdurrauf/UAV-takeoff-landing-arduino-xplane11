# -*- coding: utf-8 -*-
"""
Created on Wed Apr 02 21:24:46 2025

@author: ammar
"""

from utils import switch_tab, set_camera_behind
from uavs import UAVs
import xpc
import time

# set the xpc client
xplane = xpc.XPlaneConnect()

initial_heading = xplane.getDREF('sim/flightmodel/position/psi')[0]

# get the user UAV initial position
coordinates = xplane.getDREFs(['sim/flightmodel/position/local_x',
                               'sim/flightmodel/position/local_y',
                               'sim/flightmodel/position/local_z'])

def send_control(client, control_values = [],  ac=0,
                 elevator=-998, aileron=-998, rudder=-998, 
                 throttle=-998, gear=-998, flaps=-998):
    
    if len(control_values) == 0:
        controls = [elevator, aileron, rudder, throttle, gear, flaps]
    else:
        controls = control_values            
        
    try:
        client.sendCTRL(controls, ac)
    except:
        pass

def stabilize_flight(client, 
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
    
    send_control(client, 
                 elevator=elevator_value,
                 aileron=aileron_value, 
                 rudder=rudder_value)
    
    
    return rudder_value, aileron_value, elevator_value, accumulated_pitch_error


switch_tab()
time.sleep(1)
set_camera_behind()
time.sleep(0.5)

xplane = xpc.XPlaneConnect()


drefs = ['sim/flightmodel2/position/y_agl', # altitude ~ 0
         'sim/flightmodel/position/local_x', # position x ~ 1
         'sim/flightmodel/position/local_z', # position z (position y is the altitude) ~ 2
         'sim/flightmodel/position/indicated_airspeed', # Knots Indicated Airspeed (KIAS) ~ 3
         'sim/flightmodel/position/vh_ind', # vertical velocity ~ 4
         'sim/flightmodel/position/theta', # pitch angle ~ 5
         'sim/flightmodel/position/phi', # roll angle ~ 6
         'sim/flightmodel/position/psi', # yaw angle ~ 7
         'sim/flightmodel/position/Q', # pitch rate ~ 8
         'sim/flightmodel/position/P', # roll rate ~ 9
         'sim/flightmodel/position/R', # yaw rate ~ 10
         'sim/flightmodel2/misc/has_crashed', # crash or not ~ 11
         ]


has_crashed = 0.0
X = -998
target_altitude = 25
accumulated_pitch_error = 0

phase = 'takeoff'
set_engine_and_flap_takeoff = True
cruise_i = 0

while not has_crashed and cruise_i < 20:
    
    # retrieve flight data
    flight_data = xplane.getDREFs(drefs)
    altitude = flight_data[0][0]
    # pos_x = flight_data[1][0]
    # pos_y = flight_data[2][0]
    speed_kias = flight_data[3][0]
    pitch = flight_data[5][0]
    pitch_rate = flight_data[8][0]
    roll = flight_data[6][0]
    roll_rate = flight_data[9][0]
    yaw = flight_data[7][0]
    yaw_rate = flight_data[10][0]
    has_crashed = flight_data[11][0]
    
    if phase == 'takeoff':

        rudder, aileron, elevator, _ = stabilize_flight(xplane, 
                                                        yaw, yaw_rate,
                                                        roll, roll_rate,
                                                        yaw_target=initial_heading)

        elevator = 0.0
        if set_engine_and_flap_takeoff:
            send_control(xplane, throttle=1.0, flaps=0.2)
            set_engine_and_flap_takeoff = False

        if speed_kias >= 18.0 and speed_kias < 20.0:
            send_control(xplane, elevator=0.4)
            elevator = 0.4
            
        if speed_kias > 20.0:
            phase = 'climb'
            elevator = 1.0

        
    elif phase == 'climb':
        
        if altitude < target_altitude:
            rudder, aileron, elevator, _ = stabilize_flight(xplane, 
                                                            yaw, yaw_rate,
                                                            roll, roll_rate,
                                                            pitch, pitch_rate,
                                                            pitch_target=10,
                                                            yaw_target=initial_heading)
            
        else:
            phase = 'cruise'
            send_control(xplane, throttle=0.4, flaps=0.0)
            # send_control(xplane, elevator=0.0, throttle=0.4, flaps=0.0)
            # elevator = 0.0
    
    elif phase == 'cruise':
        rudder, aileron, elevator, accumulated_pitch_error = stabilize_flight(xplane, 
                                           yaw, yaw_rate,
                                           roll, roll_rate,
                                           pitch, pitch_rate,
                                           accumulated_pitch_error=accumulated_pitch_error,
                                           yaw_target=initial_heading)
        cruise_i += 1
            
            
    text_data = f"""
----- flight_data -----
PHASE:          {phase}
altitude:       {"{:.3f}".format(altitude)}
speed_kias:     {"{:.3f}".format(speed_kias)}
pitch:          {"{:.3f}".format(pitch)}
pitch_rate:     {"{:.3f}".format(pitch_rate)}
roll:           {"{:.3f}".format(roll)}
roll_rate:      {"{:.3f}".format(roll_rate)}
yaw:            {"{:.3f}".format(yaw)}
yaw_rate:       {"{:.3f}".format(yaw_rate)}
-----------------------
rudder:         {"{:.3f}".format(rudder)}
aileron:        {"{:.3f}".format(aileron)}
elevator:       {"{:.3f}".format(elevator)}
"""
        
    print(text_data)
    
    time.sleep(0.1)


### landing phase

drefs_landing = [
    'sim/flightmodel2/position/y_agl',  # altitude ~ 0
    'sim/flightmodel/position/indicated_airspeed',  # Knots Indicated Airspeed (KIAS) ~ 1
    'sim/flightmodel/position/theta',  # pitch angle ~ 2
    'sim/flightmodel/position/phi',  # roll angle ~ 3
    'sim/flightmodel/position/psi',  # yaw angle ~ 4
    'sim/flightmodel/position/Q',  # pitch rate ~ 5
    'sim/flightmodel/position/P',  # roll rate ~ 6
    'sim/flightmodel/position/R',  # yaw rate ~ 7
    'sim/flightmodel2/misc/has_crashed',  # crash or not ~ 8
]

phase = 'approach'
target_altitude = 10.0  # Descend to 10 meters before final landing
flare_altitude = 2.0  # Flare at 2 meters
landing_complete = False

while not landing_complete:
    flight_data = xplane.getDREFs(drefs_landing)
    altitude, speed_kias, pitch, roll, yaw, pitch_rate, roll_rate, yaw_rate, has_crashed = [fd[0] for fd in flight_data]
    
    if has_crashed:
        print("Crashed! Ending simulation.")
        break
    
    if phase == 'approach':
        send_control(xplane, throttle=0.25, flaps=0.5)  # Reduce throttle, extend flaps
        stabilize_flight(xplane, yaw, yaw_rate, roll, roll_rate, pitch, pitch_rate,
                         yaw_target=initial_heading, roll_target=0, pitch_target=-5)
        if altitude <= target_altitude:
            phase = 'final_descent'
    
    elif phase == 'final_descent':
        send_control(xplane, throttle=0.1)  # Further reduce throttle
        stabilize_flight(xplane, yaw, yaw_rate, roll, roll_rate, pitch, pitch_rate,
                         yaw_target=initial_heading, roll_target=0, pitch_target=-7)
        if altitude <= flare_altitude:
            phase = 'flare'
    
    elif phase == 'flare':
        send_control(xplane, throttle=0.08)  # Cut throttle
        stabilize_flight(xplane, yaw, yaw_rate, roll, roll_rate, pitch, pitch_rate,
                         yaw_target=initial_heading, roll_target=0, pitch_target=2)  # Slightly nose up
        if altitude < 0.5:  # Touchdown
            phase = 'landing'
    
    elif phase == 'landing':
        send_control(xplane, throttle=0.0, flaps=0.0, gear=1)  # Deploy landing gear
        print("Landing complete.")
        landing_complete = True
    
    print(f"PHASE: {phase}, ALT: {altitude:.2f}, SPD: {speed_kias:.2f}, PITCH: {pitch:.2f}")
    time.sleep(0.1)
    
