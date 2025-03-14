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

#import sys
#import threading
import time as t
#import select
import numpy as np
import cv2 as cv
from datetime import *
from setproctitle import setproctitle
# robot function
from spose import pose
from sir import ir
from srobot import robot
from scam import cam
from sedge import edge
from sgpio import gpio
from scam import cam
from uservice import service


# set title of process, so that it is not just called Python
setproctitle("mqtt-client")

############################################################


def imageAnalysis(save):
  if cam.useCam:
    ok, img, imgTime = cam.getImage()
    if not ok: # size(img) == 0):
      if cam.imageFailCnt < 5:
        print("% Failed to get image.")
    else:
      h, w, ch = img.shape
      if not service.args.silent:
        # print(f"% At {imgTime}, got image {cam.cnt} of size= {w}x{h}")
        pass
      edge.paint(img)
      if not gpio.onPi:
        cv.imshow('frame for analysis', img)
      if save:
        fn = f"image_{imgTime.strftime('%Y_%b_%d_%H%M%S_')}{cam.cnt:03d}.jpg"
        cv.imwrite(fn, img)
        if not service.args.silent:
          print(f"% Saved image {fn}")
      pass
    pass
  pass

############################################################

stateTime = datetime.now()
def stateTimePassed():
  return (datetime.now() - stateTime).total_seconds()

############################################################


def loop():
  from ulog import flog  # Import flog from ulog module
  state = 0  # Initialize state
  turns = 0  # Initialize turn counter
  timeStamp = pose.tripBtimePassed()  # Initialize timeStamp
  images = 0  # Initialize image counter
  ledon = True  # Initialize LED state
  tripTime = datetime.now()  # Initialize tripTime with current time
  oldstate = -1  # Initialize oldstate
  
  ###service.send("robobot/cmd/T0/servo", "1 5000 200")

  
  toggle = 0

  # Set up GPIO pins
  
  if not service.args.now:
    print("% Ready, press start button")
    service.send(service.topicCmd + "T0/leds","16 30 30 0")  # Set LED to yellow (waiting)
  # Main state machine
  #edge.lineControl(0, 0)  # Make sure line control is off
  while not (service.stop or gpio.stop()):  # Main loop
    if state == 0:  # Wait for start signal
      #start = gpio.start() or service.args.now
      start = 1
      print(start)
      print("% Starting trying")
      if start:
        print("% Starting")
        
        service.send(service.topicCmd + "T0/leds","16 0 0 30")  # Set LED to blue (running)
        #service.send(service.topicCmd + "ti/rc","0.5 0")  # Set robot speed and turn rate
        #edge.lineControl(0.25, 0)  # Follow line at 0.25 m/s
        print("% trying Linefollow on teensy")
        service.send(service.topicCmd + "T0/linfON", " ")
        print("% Starting Linefollow on teensy")
        state = 12  # Change state to following line
        pose.tripBreset()  # Reset trip counter/timer B
    elif state == 12:  # Following line
      if pose.tripBtimePassed() > 3:
        #pose.printPose()
        # No more line
        #edge.lineControl(0,0)  # Stop following line
        
        ###service.send("robobot/cmd/T0/servo", "1 -400 200")
        print("Moving servo")
        
       
        pose.tripBreset()
        #timeStamp = pose.tripBtimePassed() 
        #service.send(service.topicCmd + "ti/rc","0.0 0.25")  # Turn left
        #print("% turning")
        state = 14  # Change state to turning left
      print(f"% --- state {state}, h = {pose.tripBh:.4f}, t={pose.tripBtimePassed():.3f}")
    elif state == 14:  # Turning left
      if pose.tripBtimePassed() > 3: 
        turns += 1  # Increment turn counter
        
        #print(edge.crossingLine)
        #print(edge.crossingLineCnt)
        #print(edge.edge_n)
        
        print(edge.position)
        
        ###service.send("robobot/cmd/T0/servo", "1 800 200")  
        print("Moving servo")
        
        #gpio.set_value(20, 1)
        #gpio.set_value(21, 0)
      
        #gpio.set_value(26, 1)
        
        # service.send(service.topicCmd + "ti/rc","0.5 0") #Command to start driving (in this case 0.5m/s, 0 degr)
        
        #gpio.set_value(19, 0)
       
        pose.tripBreset()
        #timeStamp = pose.tripBtimePassed() 
        if turns < 3:
          state = 12
        else:
          state = 20  # Change state to image analysis
        #service.send(service.topicCmd + "ti/rc","0 0")  # Stop for images
      #print(f"% --- state {state}, h = {pose.tripBh:.4f}, t={pose.tripBtimePassed():.3f}")
    elif state == 20:  # Image analysis
      
      #gpio.set_value(20, 0)
      #gpio.set_value(21, 0)
        
      #gpio.set_value(26, 0)
      #gpio.set_value(19, 0)
      #edge.lineControl(0, 0)  # Follow line at 0.25 m/s
   
      #imageAnalysis(images == 2)  # Perform image analysis
      images += 1  # Increment image counter
      # Blink LED
      if ledon:
        service.send(service.topicCmd + "T0/leds","16 0 64 0")  # Set LED to green
        #gpio.set_value(20, 1)  # Turn on GPIO pin 20
      else:
        service.send(service.topicCmd + "T0/leds","16 0 30 30")  # Set LED to yellow
        #gpio.set_value(20, 0)  # Turn off GPIO pin 20
      ledon = not ledon  # Toggle LED state
      # Check if finished
      if images >= 10 or (not cam.useCam) or stateTimePassed() > 20:
        images = 0  # Reset image counter
        state = 99  # Change state to finished
      pass
    else:  # Abort
      print(f"% Mission finished/aborted; state={state}")
      break
    # Allow OpenCV to handle imshow (if in use)
    if (cam.useCam):
      imageAnalysis(False)  # Perform image analysis without saving
      key = cv.waitKey(100)  # Wait for key press
      if key > 0:  # If key pressed
        break
    # Note state change and reset state timer
    if state != oldstate:
      flog.write(state)  # Log state change
      flog.writeRemark(f"% State change from {oldstate} to {state}")
      print(f"% State change from {oldstate} to {state}")
      oldstate = state  # Update oldstate
      stateTime = datetime.now()  # Reset stateTime
    # Do not loop too fast
    t.sleep(0.1)  # Sleep for 0.1 seconds
    # Tell interface that we are alive
    service.send(service.topicCmd + "ti/alive",str(service.startTime))
    pass  # End of while loop
  # End of mission, turn LEDs off and stop
  service.send(service.topicCmd + "T0/leds","16 0 0 0")  # Turn off LEDs
  gpio.set_value(20, 0)  # Turn off GPIO pin 20
  #edge.lineControl(0,0)  # Stop following line
  service.send(service.topicCmd + "ti/rc","0 0")  # Stop robot
  t.sleep(0.05)  # Sleep for 0.05 seconds
  pass
############################################################

if __name__ == "__main__":
    print("% Starting")

    # where is the MQTT data server:
    service.setup('localhost') # localhost
    #service.setup('10.197.217.81') # Juniper
    #service.setup('10.197.217.80') # Newton
    #service.setup('10.197.218.172')
    if service.connected:
      loop()
      service.terminate() 
    print("% Main Terminated")
