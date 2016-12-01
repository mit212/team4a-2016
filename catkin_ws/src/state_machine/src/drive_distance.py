import rospy
import numpy as np
import tf
import tf.transformations as tfm
import sys
import traceback
import cv2
from cv_bridge import CvBridge, CvBridgeError

from me212cv.msg import DetectedObject
from me212base.msg import WheelVelCmd, ArduinoData
from apriltags.msg import AprilTagDetections
from sensor_msgs.msg import Image, CameraInfo
import me212helper.helper as helper

import drive_tag_distance
#from prepare_catch import PrepareToCatch
from state import State
from stop import Stop
from release import Release
import search

class DriveDistance(State):
    def __init__(self, current_input):
        self.current_input = current_input
        #current_input -> tuple, (distance, speed, next_state, next_input)

        self.arrived = False

        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)
        self.arduino_data_sub = rospy.Subscriber("/arduino_data", ArduinoData, self.arduino_data_callback, queue_size = 1)

        self.enc_x = 0
        self.enc_y = 0
        self.accum_x = []
        self.accum_y = []
 
        self.dist = 0
        self.desired_dist = self.current_input[0]
        self.speed = self.current_input[1]

        rospy.sleep(0.5)
        self.start_enc_x = self.enc_x
        self.start_enc_y = self.enc_y

    def run(self):
        wv = WheelVelCmd()
        wv.desiredWrist = 0

        if self.dist < self.desired_dist:
            wv.desiredWV_R = self.speed
            wv.desiredWV_L = self.speed
        else:
            wv.desiredWV_R = 0
            wv.desiredWV_L = 0
            self.arrived = True

        self.dist = np.sqrt((self.enc_x - self.start_enc_x)**2 + (self.enc_y - self.start_enc_y)**2)
        self.velcmd_pub.publish(wv)
        rospy.sleep(.01)
        
    
    def next_input(self):
        return self.current_input[3]

    def next_state(self):
        input_state = self.current_input[2] #this is a string
        if input_state == "Drive":
            return Drive(self.next_input())
        elif input_state == "Search":
            return search.Search(self.next_input())
        elif input_state == "PrepareToCatch":
            return PrepareToCatch(self.next_input())
        elif input_state == "DriveUntilTagDistance":
            return drive_tag_distance.DriveUntilTagDistance(self.next_input())
        else:
            print "DriveDistance --> Stop()"
            return Stop(self.next_input())

    def is_finished(self):
        return self.arrived

    def is_stop_state(self):
        return False

    def arduino_data_callback(self, data):
        if len(self.accum_x) >= 3:
            self.accum_x.pop(0)
            self.accum_y.pop(0)

        self.accum_x.append(data.deltaX)
        self.accum_y.append(data.deltaY)    

        self.enc_x = sum(self.accum_x)/len(self.accum_x)
        self.enc_y = sum(self.accum_y)/len(self.accum_y)

    def __str__(self):
        return "DriveDistance()" 