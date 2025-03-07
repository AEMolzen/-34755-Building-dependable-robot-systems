#!/usr/bin/env python3

# /***************************************************************************
# *   Copyright (C) 2024 by DTU
# *   jcan@dtu.dk
# *
# *
# * The MIT License (MIT)  https://mit-license.org/
# *
# * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
# * and associated documentation files (the “Software”), to deal in the Software without restriction,
# * including without limitation the rights to use, copy, modify, merge, publish, distribute,
# * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
# * is furnished to do so, subject to the following conditions:
# *
# * The above copyright notice and this permission notice shall be included in all copies
# * or substantial portions of the Software.
# *
# * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# * THE SOFTWARE. */

import time as t
import numpy as np
import cv2 as cv
from datetime import datetime
from setproctitle import setproctitle
import math
import argparse

# Robot function imports
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from uservice import service

# Set title of process
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
    """Drives the robot forward at the specified speed for the given duration."""
    service.send(service.topicCmd + "ti/rc", f"{speed} 0")  # No turning
    t.sleep(duration)
    service.send(service.topicCmd + "ti/rc", "0 0")  # Stop
    t.sleep(0.1)


def turn(turn_angle_rad):
    """Turns the robot by the specified angle (in radians)."""
    service.send(service.topicCmd + "ti/rc", f"0 {turn_angle_rad}")
    t.sleep(0.1)
    # service.send(service.topicCmd + "ti/rc", "0 0")  # Stop (might not be needed)
    # t.sleep(0.1)


def execute_square(side_length_meters, turn_direction="clockwise"):
    """Drives the robot in a square pattern with the given side length and turn direction."""

    print(f"Starting UMBmark test - Side Length: {side_length_meters}, Direction: {turn_direction}")
    log_file.write(f"UMBmark Test - Side Length: {side_length_meters}, Direction: {turn_direction}\n")

    # Calibration is done on the teensy side.  These are *guesses*.
    forward_speed = 0.2  # m/s

    # Calculate drive duration
    drive_duration = side_length_meters / forward_speed

    for i in range(4):
        print(f"Leg {i + 1}: Driving forward {side_length_meters} meters")
        log_file.write(f"  Leg {i + 1}: Driving forward {side_length_meters} meters\n")
        drive_forward(forward_speed, drive_duration)

        if turn_direction == "clockwise":
            turn_angle_rad = -math.pi / 2
        else:
            turn_angle_rad = math.pi / 2
        print(
            f"Leg {i + 1}: Turning {turn_angle_rad} radians ({'clockwise' if turn_angle_rad < 0 else 'counterclockwise'})")
        log_file.write(
            f"  Leg {i + 1}: Turning {turn_angle_rad} radians ({'clockwise' if turn_angle_rad < 0 else 'counterclockwise'})\n")
        turn(turn_angle_rad)
        t.sleep(0.1)


def loop():
    from ulog import flog

    state = 0
    oldstate = -1
    side_length = 2.0  # meters

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
            # Perform the UMB-mark test (Clockwise)
            print("Running UMB-Mark test clockwise")
            execute_square(side_length, "clockwise")
            print("UMB-Mark test complete (clockwise).")

            # Manual Measurement Input (Clockwise)
            print("Enter manual measurements for CLOCKWISE run:")
            x_error_cw = float(input("  X error (meters): "))
            y_error_cw = float(input("  Y error (meters): "))
            theta_error_cw = float(input("  Theta error (radians): "))
            log_file.write(f"Clockwise - X Error: {x_error_cw}, Y Error: {y_error_cw}, Theta Error: {theta_error_cw}\n")

            # Perform the UMB-mark test (Counterclockwise)
            print("Running UMB-Mark test counterclockwise")
            execute_square(side_length, "counterclockwise")
            print("UMB-Mark test complete (counterclockwise).")

            # Manual Measurement Input (Counterclockwise)
            print("Enter manual measurements for COUNTERCLOCKWISE run:")
            x_error_ccw = float(input("  X error (meters): "))
            y_error_ccw = float(input("  Y error (meters): "))
            theta_error_ccw = float(input("  Theta error (radians): "))
            log_file.write(
                f"Counterclockwise - X Error: {x_error_ccw}, Y Error: {y_error_ccw}, Theta Error: {theta_error_ccw}\n")

            print("UMBmark test complete.  Enter errors manually.")
            state = 2  # go to state 2 to ensure it only runs once.

        elif state == 2:
            print(f"% Mission finished/aborted; state={state}")
            break  # Exit the loop

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

    service.send(service.topicCmd + "T0/leds", "16 0 0 0")
    gpio.set_value(20, 0)
    service.send(service.topicCmd + "ti/rc", "0 0")
    t.sleep(0.05)


############################################################

if __name__ == "__main__":
    print("% Starting")

    # Argument parsing for log file name
    parser = argparse.ArgumentParser(description="UMBmark test client.")
    parser.add_argument("--log", type=str, default="umbmark_log.txt", help="Log file name.")
    args = parser.parse_args()

    # Open the log file
    log_file = open(args.log, "w")
    log_file.write(f"UMBmark Test Log - Started at {datetime.now()}\n")

    service.setup('localhost')
    if service.connected:
        loop()
        service.terminate()

    # Close the log file
    log_file.close()
    print("% Main Terminated")