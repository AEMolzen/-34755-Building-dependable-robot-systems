#!/usr/bin/env python3

import time as t
import numpy as np
import cv2 as cv
from datetime import datetime
from setproctitle import setproctitle
import math
import argparse

# Robot function imports
from spose import pose  # For odometry
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

def turn(target_angle_rad):
    """Turns the robot to a target angle using a PI controller."""
    initial_angle = pose.pose[2]  # Use pose.pose[2] for yaw
    print(f"Initial angle: {initial_angle:.3f}")
    current_angle = initial_angle

    # PI Controller Gains (Tune these!)
    Kp = 2  # Proportional gain
    Ki = 0.5   # Integral gain

    integral_error = 0.0
    last_time = t.time()

    while abs(current_angle - target_angle_rad) > 0.01:  # Tolerance
        current_angle = pose.pose[2]  # Use pose.pose[2] for yaw
        dt = t.time() - last_time
        last_time = t.time()

        # Calculate error
        error = target_angle_rad - current_angle

        # Wrap error to -pi to pi
        error = (error + math.pi) % (2 * math.pi) - math.pi

        # Accumulate integral error
        integral_error += error * dt

        # Calculate control output (turn rate)
        turn_rate = Kp * error + Ki * integral_error

        # Limit turn rate (Important!)
        max_turn_rate = 2.0  # rad/s (Adjust as needed)
        turn_rate = max(min(turn_rate, max_turn_rate), -max_turn_rate)

        # Send turn command
        service.send(service.topicCmd + "ti/rc", f"0 {turn_rate}")
        #print(f"Cur: {current_angle:.3f}, Tar: {target_angle_rad:.3f}, Err: {error:.3f}, Rate: {turn_rate:.3f}")
        t.sleep(0.01)

    service.send(service.topicCmd + "ti/rc", "0 0")  # Stop
    t.sleep(0.5)


def execute_square(side_length_meters, turn_direction="clockwise"):
    """Drives a square, using PI control for turns."""
    print(f"Starting UMBmark - Side Length: {side_length_meters}, Direction: {turn_direction}")
    log_file.write(f"UMBmark Test - Side: {side_length_meters}, Direction: {turn_direction}\n")

    forward_speed = 0.1  # m/s

    drive_duration = side_length_meters / forward_speed

    for i in range(4):
        print(f"Leg {i+1}: Driving forward {side_length_meters} meters")
        log_file.write(f"  Leg {i+1}: Driving forward {side_length_meters} meters\n")
        drive_forward(forward_speed, drive_duration)

        initial_yaw = pose.pose[2]  # Use pose.pose[2]
        if turn_direction == "clockwise":
            target_yaw = initial_yaw - (math.pi / 2)
        else:
            target_yaw = initial_yaw + (math.pi / 2)

        # Wrap to -pi to pi
        target_yaw = (target_yaw + math.pi) % (2 * math.pi) - math.pi

        print(f"Leg {i+1}: Turning {'clockwise' if turn_direction == 'clockwise' else 'counterclockwise'} to {target_yaw:.3f} rad")
        log_file.write(f"  Leg {i+1}: Turning {'cw' if turn_direction == 'clockwise' else 'ccw'} to {target_yaw:.3f} rad\n")
        turn(target_yaw)


def loop():
    from ulog import flog

    state = 0
    oldstate = -1
    side_length = 1.0

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
                #print(pose.__dict__) # REMOVE THIS - no longer needed

        elif state == 1:
            # Run Clockwise
            execute_square(side_length, "clockwise")
            print("UMB-Mark test complete (clockwise).")
            service.send(service.topicCmd + "ti/rc", "0 0")
            t.sleep(0.5)

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
    parser.add_argument("--log", type=str, default="umbmark_log.txt")
    parser.add_argument("--now", action="store_true")
    args = parser.parse_args()

    log_file = open(args.log, "w")
    log_file.write(f"UMBmark Test Log - Started at {datetime.now()}\n")

    service.setup('localhost')
    if service.connected:
        loop()
        service.terminate()

    log_file.close()
    print("% Main Terminated")