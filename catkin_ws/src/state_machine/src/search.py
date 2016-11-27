import rospy
import tf
import numpy as np
import threading
import serial
import pdb
import traceback
import sys
import tf.transformations as tfm

from me212base.msg import WheelVelCmd
from apriltags.msg import AprilTagDetections
import me212helper.helper as helper

import detect_obstacles
from state import State
from stop import Stop
from drive import Drive

class Search(State):
    def __init__(self, current_input):
        self.current_input = current_input
        self.found_target = False
        self.tags_in_view = []
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)

        self.detect_obstacles_next = False
        if current_input - int(current_input) != 0:
            self.detect_obstacles_next = True
            self.current_input = int(current_input)

        self.right_turns = [2]
        self.left_turns = [0]

        self.classified_obstacles = False
        
    def run(self):
        wv = WheelVelCmd()

        if self.current_input in self.tags_in_view:
            print "found target"
            wv.desiredWV_R = 0  # right, left
            wv.desiredWV_L = 0
            self.found_target = True
        elif self.current_input in self.right_turns:
            wv.desiredWV_R = -0.05  # right, left
            wv.desiredWV_L = 0.05
        else:
            # turn left
            wv.desiredWV_R = 0.05  # right, left
            wv.desiredWV_L = -0.05

        self.velcmd_pub.publish(wv)
    
    def next_input(self):
        return self.current_input # maybe change later

    # update
    def next_state(self):
        if self.detect_obstacles_next:
            return detect_obstacles.DetectObstacles(self.next_input())
        return Drive(self.current_input)

    def is_finished(self):
        return self.found_target

    def is_stop_state(self):
        return False
        
    def apriltag_callback(self, data):
        del self.tags_in_view[:]
        for detection in data.detections:
            #print detection.pose.position.x, detection.pose.position.y, detection.pose.position.z
            self.tags_in_view.append(detection.id)

    def __str__(self):
        return "Search(%s)" % (self.current_input)
