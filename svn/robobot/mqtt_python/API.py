
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

def drive_turn(speed, turn):

    par = f"{speed:.3f} {turn:.3f}"
    service.send(service.topicCmd + "ti/rc", par) # send new turn command, maintaining velocity
    pass

def Servo(angle, anglespeed):

    par = f"{angle:.3f} {anglespeed:.3f}"
    service.send(service.topicCmd + "t0/servo", par) # send new turn command, maintaining velocity
    pass

def Servo_OFF():

    par = f"{5000:.3f} {5000:.3f}"
    service.send(service.topicCmd + "t0/servo", par) # send new turn command, maintaining velocity
    pass