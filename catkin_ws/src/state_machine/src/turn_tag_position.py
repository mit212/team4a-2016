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

from prepare_catch import PrepareToCatch
from drive_tag_distance import DriveUntilTagDistance
from state import State
from stop import Stop
from release import Release
import search

class TurnTagPosition(State):
    def __init__(self, current_input):
        self.current_input = current_input
        #current_input -> tuple, (tag_id, desired_pos_in_frame, direction)

        self.tag_id = current_input[0]
        self.desired_pos_in_frame = current_input[1]
        self.turn_direction = current_input[2]
        self.arrived = False

        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)

        self.tags_in_view = []
        self.tag_centers = {}

        self.pixel_threshold = 5

        self.TURN_RIGHT = 1
        self.TURN_LEFT = -self.TURN_RIGHT

        rospy.sleep(0.5)

    def run(self):
        wv = WheelVelCmd()
        wv.desiredWrist = 0

        if self.tag_id in self.tags_in_view:
            tag_center_x = self.tag_centers[self.tag_id][0]
            print "tag_center_x", tag_center_x
            if tag_center_x <= self.desired_pos_in_frame - self.pixel_threshold:
                wv.desiredWV_R = 0.1
                wv.desiredWV_L = 0
            elif tag_center_x >= self.desired_pos_in_frame + self.pixel_threshold:
                wv.desiredWV_R = 0
                wv.desiredWV_L = 0.1
            else:
                wv.desiredWV_R = 0
                wv.desiredWV_L = 0
                self.arrived = True
        else:
            if self.turn_direction == self.TURN_LEFT: #was TL
                wv.desiredWV_R = 0.1
                wv.desiredWV_L = 0 
            elif self.turn_direction == self.TURN_RIGHT: #was TR
                wv.desiredWV_R = 0
                wv.desiredWV_L = 0.1
            else:
                print "kbai"

        self.velcmd_pub.publish(wv)    
        rospy.sleep(0.01)

    def next_input(self):
        next_distance = 0
        if self.tag_id == 0:
            next_distance = 0.60
        elif self.tag_id == 2:
            if rospy.get_param("field_has_far_obstacles"):
                next_distance = 1.45 # far obstacles
            else:
                next_distance = 0.70 # near obstacles
        elif self.tag_id == 4:
            next_distance = 0.7
        elif self.tag_id == 7:
            next_distance = 1.15

        return (self.tag_id, next_distance)

    def next_state(self):
        return DriveUntilTagDistance(self.next_input())

    def is_finished(self):
        return self.arrived

    def is_stop_state(self):
        return False

    def apriltag_callback(self, data):
        del self.tags_in_view[:]
        for detection in data.detections:
            corner_x = []
            corner_y = []
            self.tags_in_view.append(detection.id)
            for point in detection.corners2d:
                corner_x.append(point.x)
                corner_y.append(point.y)
            center_x = sum(corner_x) / len(corner_x)
            center_y = sum(corner_y) / len(corner_y)
            self.tag_centers[detection.id] = (center_x, center_y)

    def __str__(self):
        return "TurnUntilTagInPosition(%s, %s, %s)" % (self.tag_id, self.desired_pos_in_frame, self.turn_direction) 