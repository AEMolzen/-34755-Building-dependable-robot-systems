#!/usr/bin/env python3

import time as t
import numpy as np
import cv2 as cv
from datetime import datetime
from setproctitle import setproctitle
import math
import argparse

# Robot function imports
from spose import pose  # We'll use spose for odometry
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from uservice import service

# Set process title
setproctitle("mqtt-client")

############################################################

def imageAnalysis(save):
    if cam.useCam:
        ok, img, imgTime = cam.getImage()
        if not ok:
            if cam.imageFailCnt < 5:
                print("% Failed to get image.")
        else:
            h, w, ch = img.shape
            if not service.args.silent:
                pass
            edge.paint(img)
            if not gpio.onPi:
                cv.imshow('frame for analysis', img)
            if save:
                fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
                cv.imwrite(fn, img)
                if not service.args.silent:
                    print(f"% Saved image {fn}")

############################################################

stateTime = datetime.now()

def stateTimePassed():
    return (datetime.now() - stateTime).total_seconds()

############################################################

def drive_forward(speed, duration):
    """Drives the robot forward."""
    service.send(service.topicCmd + "ti/rc", f"{speed} 0")
    t.sleep(duration)
    service.send(service.topicCmd + "ti/rc", "0 0")
    t.sleep(0.5)

def turn(turn_rate_rad_per_sec, target_angle_rad):
    """Turns the robot to a target angle using odometry."""
    initial_angle = pose.yaw  # Get initial orientation from spose
    print(f"Initial angle: {initial_angle:.3f}")
    current_angle = initial_angle
    
    service.send(service.topicCmd + "ti/rc", f"0 {turn_rate_rad_per_sec}")

    while abs(current_angle - target_angle_rad) > 0.02:  # Tolerance of 0.02 radians (~1 degree)
        current_angle = pose.yaw # Get the updated angle
        #print(f"Current angle: {current_angle:.3f}, Target: {target_angle_rad:.3f}")
        t.sleep(0.01)  # Short delay to avoid overwhelming the system

    service.send(service.topicCmd + "ti/rc", "0 0")  # Stop turning
    t.sleep(0.5)

def execute_square(side_length_meters, turn_direction="clockwise"):
    """Drives the robot in a square."""
    print(f"Starting UMBmark test - Side Length: {side_length_meters}, Direction: {turn_direction}")
    log_file.write(f"UMBmark Test - Side Length: {side_length_meters}, Direction: {turn_direction}\n")

    forward_speed = 0.1 # m/s
    turn_rate_rad_per_sec = 0.3  # rad/s  (Still needs calibration, but less critical)

    drive_duration = side_length_meters / forward_speed

    for i in range(4):
        print(f"Leg {i+1}: Driving forward {side_length_meters} meters")
        log_file.write(f"  Leg {i+1}: Driving forward {side_length_meters} meters\n")
        drive_forward(forward_speed, drive_duration)

        initial_yaw = pose.yaw  # Record initial yaw before turning
        if turn_direction == "clockwise":
            target_yaw = initial_yaw - (math.pi / 2)
            turn_rate = -turn_rate_rad_per_sec #Clockwise
        else:
            target_yaw = initial_yaw + (math.pi / 2)
            turn_rate = turn_rate_rad_per_sec #Counter-Clockwise

        # Ensure target_yaw is within -pi to pi range
        target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi

        print(f"Leg {i+1}: Turning {'clockwise' if turn_rate < 0 else 'counterclockwise'} to {target_yaw:.3f} radians")
        log_file.write(f"  Leg {i+1}: Turning {'clockwise' if turn_rate < 0 else 'counterclockwise'} to {target_yaw:.3f} radians\n")
        turn(turn_rate, target_yaw)


def loop():
    from ulog import flog

    state = 0
    oldstate = -1
    side_length = 1.0  # meters

    if not service.args.now:
        print("% Ready, press start button")
        service.send(service.topicCmd + "T0/leds", "16 30 30 0")

    edge.lineControl(0, 0)  # Disable line following

    while not (service.stop or gpio.stop()):
        if state == 0:
            start = gpio.start() or service.args.now
            if start:
                print("% Starting")
                service.send(service.topicCmd + "T0/leds", "16 0 0 30")
                state = 1
                pose.tripBreset()

        elif state == 1:
            # Run UMBmark test - Clockwise and Counterclockwise
            execute_square(side_length, "clockwise")
            print("UMB-Mark test complete (clockwise).")

            # Get manual measurements (Clockwise) -  Still needed for overall calibration!
            print("Enter manual measurements for CLOCKWISE run:")
            x_error_cw = float(input("  X error (meters): "))
            y_error_cw = float(input("  Y error (meters): "))
            theta_error_cw = float(input("  Theta error (radians): "))
            log_file.write(f"Clockwise - X Error: {x_error_cw}, Y Error: {y_error_cw}, Theta Error: {theta_error_cw}\n")
            log_file.flush()

            execute_square(side_length, "counterclockwise")
            print("UMB-Mark test complete (counterclockwise).")

            # Get manual measurements (Counterclockwise)
            print("Enter manual measurements for COUNTERCLOCKWISE run:")
            x_error_ccw = float(input("  X error (meters): "))
            y_error_ccw = float(input("  Y error (meters): "))
            theta_error_ccw = float(input("  Theta error (radians): "))
            log_file.write(f"Counterclockwise - X Error: {x_error_ccw}, Y Error: {y_error_ccw}, Theta Error: {theta_error_ccw}\n")
            log_file.flush()

            print("UMBmark test complete. Data logged.")
            state = 2

        elif state == 2:
            print(f"% Mission finished/aborted; state={state}")
            break

        if cam.useCam:
            imageAnalysis(False)
            key = cv.waitKey(100)
            if key > 0:
                break

        if state != oldstate:
            flog.write(state)
            flog.writeRemark(f"% State change from {oldstate} to {state}")
            print(f"% State change from {oldstate} to {state}")
            oldstate = state
            stateTime = datetime.now()

        t.sleep(0.1)
        service.send(service.topicCmd + "ti/alive", str(service.startTime))

    # Cleanup
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")
    gpio.set_value(20, 0)
    service.send(service.topicCmd + "ti/rc", "0 0")
    t.sleep(0.05)

############################################################

if __name__ == "__main__":
    print("% Starting")

    parser = argparse.ArgumentParser(description="UMBmark test client.")
    parser.add_argument("--log", type=str, default="umbmark_log.txt", help="Log file name.")
    parser.add_argument("--now", action="store_true", help="Start immediately.")
    args = parser.parse_args()

    log_file = open(args.log, "w")
    log_file.write(f"UMBmark Test Log - Started at {datetime.now()}\n")

    service.setup('localhost')
    if service.connected:
        loop()
        service.terminate()

    log_file.close()
    print("% Main Terminated")