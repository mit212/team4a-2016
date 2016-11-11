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

from state import State
from stop import Stop
from drive import Drive

class Search(State):
    def __init__(self, current_input):
        self.current_input = current_input
        self.found_target = False
        self.tags_in_view = []
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        
    def run(self):
        if self.current_input in self.tags_in_view:
            print "found target"
            self.found_target = True
    
    def next_input(self):
        return self.current_input # maybe change later

    def next_state(self):
        return Drive(self.next_input())

    def is_finished(self):
        return self.found_target

    def is_stop_state(self):
        return False
        
    def apriltag_callback(self, data):
        del self.tags_in_view[:]
        for detection in data.detections:
            #print detection.pose.position.x, detection.pose.position.y, detection.pose.position.z
            self.tags_in_view.append(detection.id)
