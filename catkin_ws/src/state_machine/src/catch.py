import rospy
import tf
import threading
import serial
import pdb
import traceback
import sys
import tf.transformations as tfm

from end_effector.msg import end_effector
import me212helper.helper as helper

from state import State
from stop import Stop
from drive import Drive

#catch state manages three motions: extending the linear actuator, 
#                                   rotating the wrist 90 degrees, 
#                                   and retracting the linear actuator


class Catch(State):
    def __init__(self, current_input):
        self.current_input = current_input
        self.catch_success = False
        ##currently reading distance from apriltag, this should change to an object detection thing to determine distance to actual object
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        self.end_effector_sub = rospy.Subscriber("/end_effector/state", end_effector, self.end_effector_callback, queue_size = 1)

        
    def run(self):
        #check where the linear actuator is
        #check that the wrist is in the correct position
        #extension_controller(extended_distance)
        #set the servo position
        #extension_controller(retracted_distance)
        #once all of this is done, print "catch success" self.catch_success = True
        
        if self.current_input in self.tags_in_view:
            print "found target"
            self.found_target = True
    
    def next_input(self):
        return self.current_input # maybe change later

    def next_state(self):
        return Search(self.next_input())

    def is_finished(self):
        return self.catch_success

    def is_stop_state(self):
        return False
        
    def apriltag_callback(self, data):
        del self.tags_in_view[:]
        for detection in data.detections:
            #print detection.pose.position.x, detection.pose.position.y, detection.pose.position.z
            self.tags_in_view.append(detection.id)

    def extension_controller(self, position):
        while:
            #do something
        return

