#!/usr/bin/env python3

#/***************************************************************************
#*   Copyright (C) 2024 by DTU
#*   jcan@dtu.dk
#*
#*
#* The MIT License (MIT)  https://mit-license.org/
#*
#* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
#* and associated documentation files (the “Software”), to deal in the Software without restriction,
#* including without limitation the rights to use, copy, modify, merge, publish, distribute,
#* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
#* is furnished to do so, subject to the following conditions:
#*
#* The above copyright notice and this permission notice shall be included in all copies
#* or substantial portions of the Software.
#*
#* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
#* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
#* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
#* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
#* THE SOFTWARE. */

import time as t
import numpy as np
import cv2 as cv
from datetime import datetime
from setproctitle import setproctitle

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

def loop():
    from ulog import flog
    
    state = 0
    turns = 0
    timeStamp = pose.tripBtimePassed()
    images = 0
    ledon = True
    tripTime = datetime.now()
    oldstate = -1
    
    if not service.args.now:
        print("% Ready, press start button")
        service.send(service.topicCmd + "T0/leds", "16 30 30 0")
    
    edge.lineControl(0, 0) 
    # Turn on or off LineControl. 
    # LineControl(velocity, center), e.g. LineControl(0,0) disables, LineControl(1,0) enables.
    
    while not (service.stop or gpio.stop()):
        if state == 0:
            start = gpio.start() or service.args.now
            if start:
                print("% Starting")
                service.send(service.topicCmd + "T0/leds", "16 0 0 30")
                state = 12
                pose.tripBreset()
        
        elif state == 12:
            if pose.tripBtimePassed() > 5:
                service.send("robobot/cmd/T0/servo", "1 200 200")
                print("Moving servo")
                pose.tripBreset()
                state = 14
            print(f"% --- state {state}, h = {pose.tripBh:.4f}, t={pose.tripBtimePassed():.3f}")
        
        elif state == 14:
            if pose.tripBtimePassed() > 5:
                turns += 1
                service.send("robobot/cmd/T0/servo", "1 1000 200")
                print("Moving servo")
                pose.tripBreset()
                if turns < 3:
                    state = 12
                else:
                    state = 20  # Change state to image analysis
        
        elif state == 20:
            service.send("robobot/cmd/T0/servo", "1 5000 200")
            imageAnalysis(images == 2)
            images += 1
            
            if ledon:
                service.send(service.topicCmd + "T0/leds", "16 0 64 0")
                gpio.set_value(20, 1)
            else:
                service.send(service.topicCmd + "T0/leds", "16 0 30 30")
                gpio.set_value(20, 0)
            ledon = not ledon
            
            if images >= 10 or (not cam.useCam) or stateTimePassed() > 20:
                images = 0
                state = 99
        
        else:
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
    
    service.send(service.topicCmd + "T0/leds", "16 0 0 0")
    gpio.set_value(20, 0)
    service.send(service.topicCmd + "ti/rc", "0 0")
    t.sleep(0.05)

############################################################

if __name__ == "__main__":
    print("% Starting")
    service.setup('localhost')
    if service.connected:
        loop()
        service.terminate()
    print("% Main Terminated")
