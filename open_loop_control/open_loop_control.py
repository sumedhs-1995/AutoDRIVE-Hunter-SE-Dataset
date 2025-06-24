#!/usr/bin/env python

# Import libraries
import socketio
import eventlet
from flask import Flask
from itertools import chain
import autodrive
import numpy as np
import time
import argparse
import pandas as pd
from pynput.keyboard import Key, Listener, KeyCode
import asyncio
from scipy.interpolate import interp1d
import select # Waiting for I/O completion
import os
import sys # System-specific parameters and functions
if os.name == 'nt':
    import msvcrt # Useful routines from the MS VC++ runtime
else:
    import termios # POSIX style tty control
    import tty # Terminal control functions
from datetime import datetime
################################################################################

# Initialize vehicle(s)
v_1 = autodrive.HunterSE()
v_1.id = 'V1'

# Initialize the server
sio = socketio.Server()

# Flask (web) app
app = Flask(__name__) # '__main__'

# Throttle-Speed Mapping
V = np.array([0, 0.31818, 0.62602, 0.92693, 1.22307, 1.51606, 1.80913, 2.10907, 2.50022, 3.01602, 3.56114])
T = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
throttle2velocity = interp1d(T, V)

# Create Initial Dataframe
log = pd.DataFrame({"Timestamp":[], "Throttle (%)": [], "Steering (%)": [], "LeftEncAng (rad)": [],"RightEncAng (rad)": [], 
                    "PosX (m)": [], "PosY (m)": [], "PosZ (m)": [], "Roll (rad)": [], "Pitch (rad)": [], "Yaw (rad)": [],
                    "LinearSpeed (m/s)": [], "AngX (rad/s)": [], "AngY (rad/s)": [], "AngZ (rad/s)": [],
                    "AccX (m/s^2)": [], "AccY (m/s^2)": [], "AccZ (m/s^2)": []})

# Parameters
DRIVE_LIMIT = 0.5
STEER_LIMIT = 1.0
DRIVE_STEP_SIZE = 0.01
STEER_STEP_SIZE = 0.2

# Information
info = """
---------------------------------------
AutoDRIVE - F1TENTH Teleoperation Panel
---------------------------------------

              Q   W   E
              A   S   D
                  X
                  R

W/S : Increase/decrease drive command
D/A : Increase/decrease steer command
Q   : Zero steer
E   : Emergency brake
X   : Force stop and zero steer
R   : Soft-reset the simulator
Press CTRL+C to quit

NOTE: Press keys within this terminal
---------------------------------------
"""

# Error
error = """
ERROR: Communication failed!
"""

throttle_cmd = 0.0
steering_cmd = 0.0
# destination_path
# Data Recorder
async def DataLogger(v_1, t_teleop):
    
    global log
    next_data = pd.DataFrame({"Timestamp":[pd.Timestamp.now()], "Throttle (%)": [v_1.throttle], "Steering (%)": [v_1.steering], "LeftEncAng (rad)": [v_1.encoder_angles[0]],
                        "RightEncAng (rad)": [v_1.encoder_angles[1]], "PosX (m)": [v_1.position[0]], "PosY (m)": [v_1.position[1]], "PosZ (m)": [v_1.position[2]],
                        "Roll (rad)": [v_1.orientation_euler_angles[0]], "Pitch (rad)": [v_1.orientation_euler_angles[1]], "Yaw (rad)": [v_1.orientation_euler_angles[2]],
                        "LinearSpeed (m/s)": [throttle2velocity(v_1.throttle)],
                        "AngX (rad/s)": [v_1.angular_velocity[0]], "AngY (rad/s)": [v_1.angular_velocity[1]], "AngZ (rad/s)": [v_1.angular_velocity[2]],
                        "AccX (m/s^2)": [v_1.linear_acceleration[0]], "AccY (m/s^2)": [v_1.linear_acceleration[1]], "AccZ (m/s^2)": [v_1.linear_acceleration[2]]})
    log = pd.concat([log, next_data], ignore_index=True)


    if (time.time_ns() - t_start) <= t_teleop:
        log.to_csv("data.csv", index = False)
        print('Telelop maneuver completed')
    await asyncio.sleep(0.01)
        
        

# async def main():
#     # record_data(v_1, i, stop)
#     task = asyncio.create_task(record_data(v_1, i, stop))
#     # await task

# Registering "connect" event handler for the server
@sio.on('connect')
def connect(sid, environ):
    print('Connected!')

# Data Recording

stop = False
drive = False
brake = False
left  = False
right = False

def on_key_press(key):
    global stop, drive, brake, left, right
    if key==Key.shift:
        stop = True
    if str(key) == "'w'":
        drive = True
        brake = False
        
    if str(key) == "'a'":
        left = True
        right =False
        
    if str(key) == "'s'":
        drive = False
        brake = True
        
    if str(key) == "'d'":
        left = False
        right =True
        

def on_key_release(key):
    global left, right
    if key==Key.shift:
        print("Telelop Maneuver Completed")
    if str(key) == "'a'":
        left  = False
        right = False
    if str(key) == "'d'":
        left = False
        right = False
        

keyboard_listener = Listener(on_press=on_key_press, on_release=on_key_release)

# Registering "Bridge" event handler for the server
@sio.on('Bridge')
def bridge(sid, data):
    if data:
        v_1.parse_data(data, verbose=False)
        ###############################################################################
        if maneuver == 'teleop':
            t_teleop = 90e9 # Time for teleop maneuver
            global i
            global throttle_cmd
            global steering_cmd
            # global t, throttle_, steering_, encoder_, position_, orientation_, ang_vel, lin_acc
            
            # print(i)

            asyncio.run(DataLogger(v_1, t_teleop))

            if drive == True and brake ==False:
                throttle_cmd =  min(DRIVE_LIMIT, throttle_cmd + DRIVE_STEP_SIZE)
            elif drive == False and brake ==True:
                throttle_cmd =  max(0, throttle_cmd - DRIVE_STEP_SIZE)
            
            if left == True and right ==False:
                steering_cmd = min(STEER_LIMIT, steering_cmd+STEER_STEP_SIZE)
            elif left ==False and right ==True:
                steering_cmd = max(-STEER_LIMIT, steering_cmd-STEER_STEP_SIZE)
            
            
            if stop == True:
                throttle_cmd = 0.0
                steering_cmd = 0.0
        # Straight maneuver (constant throttle and zero steering)
        if maneuver == 'straight':
            t_straight = 90e9 # Time for straight maneuver
            # Straight
            if (time.time_ns() - t_start) <= t_straight:
                throttle_cmd = throttle + np.random.normal(0,throttle_noise) # Constant throttle (with noise)
                steering_cmd = 0 + np.random.normal(0,steering_noise) # Zero steering (with noise)
                print("Time : {:.4f} sec | Throttle : {:.2f} % | Steering : {:.4f} rad".format((time.time_ns()-t_start)/1e9, throttle_cmd*100, min(steering_cmd, 0.5236))) # Verbose
            # Stop
            else:
                throttle_cmd = 0 # Zero throttle
                steering_cmd = 0 # Zero steering
                print('Straight maneuver completed!') # Verbose
        ################################################################################
        # Skidpad maneuver (constant throttle and constant steering)
        if maneuver == 'skidpad':
            t_skidpad = 90e9 # Time for skidpad maneuver
            # Skidpad
            if (time.time_ns() - t_start) <= t_skidpad:
                throttle_cmd = throttle + np.random.normal(0,throttle_noise) # Constant throttle (with noise)
                steering_cmd = steering + np.random.normal(0,steering_noise) # Constant steering (with noise)
                print("Time : {:.4f} sec | Throttle : {:.2f} % | Steering : {:.4f} rad".format((time.time_ns()-t_start)/1e9, throttle_cmd*100, min(steering_cmd, 0.5236))) # Verbose
            # Stop
            else:
                throttle_cmd = 0 # Zero throttle
                steering_cmd = 0 # Zero steering
                print('Skidpad maneuver completed!') # Verbose
        ################################################################################
        # Fishhook maneuver (constant throttle and ramp steering)
        elif maneuver == 'fishhook':
            t_fishhook = 90e9 # Time for fishhook maneuver
            k_fishhook = 6e-12 # Controls steering rate (e.g. 1e-11 steers slower than 1e-10)
            # Fishhook
            if (time.time_ns() - t_start) <= t_fishhook:
                throttle_cmd = throttle + np.random.normal(0,throttle_noise) # Constant throttle (with noise)
                steering_cmd = k_fishhook*(time.time_ns() - t_start) + np.random.normal(0,steering_noise) # Time-dependent ramp steering (with noise)
                print("Time : {:.4f} sec | Throttle : {:.2f} % | Steering : {:.4f} rad".format((time.time_ns()-t_start)/1e9, throttle_cmd*100, min(steering_cmd, 0.5236))) # Verbose
            # Stop
            else:
                throttle_cmd = 0 # Zero throttle
                steering_cmd = 0 # Zero steering
                print('Fishhook maneuver completed!') # Verbose
        ################################################################################
        # Slalom maneuver (constant throttle and sinusoidal steering)
        elif maneuver == 'slalom':
            t_straight = 3e9 # Time for driving straight
            #t_straight = (0.5/throttle)*1e9 # Throttle-dependent time for driving straight
            t_slalom = 90e9 # Time for slalom maneuver
            #t_slalom = (5/throttle)*1e9 # Throttle-dependent time for slalom maneuver
            k_slalom = 9e-10 # Controls steering rate (e.g. 9e-10 steers slower than 1e-9)
            # Straight
            if (time.time_ns() - t_start) <= t_straight:
                throttle_cmd = throttle + np.random.normal(0,throttle_noise) # Constant throttle (with noise)
                steering_cmd = 0 + np.random.normal(0,steering_noise) # Zero steering (with noise)
                print("Time : {:.4f} sec | Throttle : {:.2f} % | Steering : {:.4f} rad".format((time.time_ns()-t_start)/1e9, throttle_cmd*100, min(steering_cmd, 0.5236))) # Verbose
            # Slalom
            elif (time.time_ns() - t_start) > t_straight and (time.time_ns() - t_start) <= (t_straight + t_slalom):
                throttle_cmd = throttle + np.random.normal(0,throttle_noise) # Constant throttle (with noise)
                steering_cmd = steering*np.cos(k_slalom*(time.time_ns() - (t_start + t_straight))) + np.random.normal(0,steering_noise) # Time-dependent sinusoidal steering (with noise)
                print("Time : {:.4f} sec | Throttle : {:.2f} % | Steering : {:.4f} rad".format((time.time_ns()-t_start)/1e9, throttle_cmd*100, min(steering_cmd, 0.5236))) # Verbose
            # Stop
            else:
                throttle_cmd = 0 # Zero throttle
                steering_cmd = 0 # Zero steering
                print('Slalom maneuver completed!') # Verbose

        
 
            # if i<N:
            #     throttle_cmd = cmd_vel[i,0]
            #     steering_cmd = cmd_vel[i,1]
            #     if stop == True:
            #         throttle_cmd = 0.0
            #         steering_cmd = 0.0   

            # else:
            #     throttle_cmd = 0.0
            #     steering_cmd = 0.0
            # 
           
        ################################################################################
        # Limit actuation
        # if steering_cmd >= 0.5236:
        #     steering_cmd = 0.5236
        # if steering_cmd <= -0.5236:
        #     steering_cmd = -0.5236
        # if throttle_cmd >= 1:
        #     throttle_cmd = 1
        # if throttle_cmd <= -1:
        #     throttle_cmd = -1
        # Direction of maneuver
        # if direction =='cw':
        #     steering_cmd = -steering_cmd # Negate steering command
        # Vehicle control mode
        v_1.cosim_mode = 0
        # Pose commands (only if cosim_mode==1)
        # v_1.posX_command = 0
        # v_1.posY_command = 0
        # v_1.posZ_command = 0.1
        # v_1.rotX_command = 0
        # v_1.rotY_command = 0
        # v_1.rotZ_command = 0
        # v_1.rotW_command = 1
        # Actuator commands (only if cosim_mode==0)
        v_1.throttle_command = throttle_cmd # [-1, 1]
        v_1.steering_command = steering_cmd # [-1, 1]
        # Publish control commands
        json_msg = v_1.generate_commands(verbose=True) # Generate vehicle 1 message
        try:
            sio.emit('Bridge', data=json_msg)
        except Exception as exception_instance:
            print(exception_instance)

################################################################################

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__) # Argument parser
    argparser.add_argument(
        '-m', '--maneuver',
        metavar='MANEUVER',
        dest='maneuver',
        choices = {'straight','skidpad','fishhook','slalom','teleop'},
        default='skidpad',
        help='select maneuver {straight, skidpad, fishhook, slalom}')
    argparser.add_argument(
        '-d', '--direction',
        metavar='DIRECTION',
        dest='direction',
        choices = {'cw','ccw'},
        default='ccw',
        help='select direction {cw, ccw}')
    argparser.add_argument(
        '-t', '--throttle',
        metavar='THROTTLE',
        dest='throttle',
        default=1,
        help='set throttle limit [-1, 1] norm%')
    argparser.add_argument(
        '-s', '--steering',
        metavar='STEERING',
        dest='steering',
        default=0.5236,
        help='set steering limit [0, 0.5236] rad')
    argparser.add_argument(
        '-tn', '--throttle_noise',
        metavar='THROTTLE_NOISE',
        dest='throttle_noise',
        default=0.0,
        help='set std dev for noisy throttle [0.001, 0.1] norm%')
    argparser.add_argument(
        '-sn', '--steering_noise',
        metavar='STEERING_NOISE',
        dest='steering_noise',
        default=0.0,
        help='set std dev for noisy steering [0.001, 0.1] rad')
    args = argparser.parse_args() # Parse the command line arguments (CLIs)
    t_start = time.time_ns() # Record starting time
    keyboard_listener.start()
    # print(time.time_ns())
    maneuver = 'teleop' # Load maneuver
    # direction = args.direction # Load maneuver direction
    throttle = float(args.throttle) # Load throttle limit
    steering = float(args.steering) # Load steering limit
    throttle_noise = float(args.throttle_noise) # Load throttle std dev
    steering_noise = float(args.steering_noise) # Load steering std dev
    settings = None
    
    
    app = socketio.Middleware(sio, app) # Wrap flask application with socketio's middleware
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app) # Deploy as an eventlet WSGI server