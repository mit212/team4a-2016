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

import prepare_catch
from state import State
from stop import Stop
from release import Release
import search
import turn_tag_position

class DriveUntilTagDistance(State):
    def __init__(self, current_input):
        self.current_input = current_input
        # current_input -> tuple, (tag_id, desired_distance_away)

        self.tag_id = current_input[0]
        self.desired_dist_away = current_input[1]
        self.arrived = False

        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)

        ####### Distance detection #######

        # Publisher for converting a CV Image to a ROS Image
        self.image_pub = rospy.Publisher('/distance_image', Image, queue_size = 1)

        # Bridge to convert ROS Image type to OpenCV Image type
        self.cv_bridge = CvBridge()

        depth_sub = rospy.Subscriber("/camera/depth_registered/image", Image, self.distance_callback, queue_size = 1)

        self.apriltag_distance = 20
        ##################################

        self.tags_in_view = []
        self.tag_centers = {}

        self.distance_threshold = .01
        self.k = 0.8

        self.stall_count = 0

        self.TURN_RIGHT = 1
        self.TURN_LEFT = -self.TURN_RIGHT

        rospy.sleep(0.5)

    def run(self):
        wv = WheelVelCmd()
        wv.desiredWrist = 0.0

        if 0.3 < self.apriltag_distance < 10:
            if self.tag_id in self.tags_in_view:
                distance_remaining = self.apriltag_distance - self.desired_dist_away

                if abs(distance_remaining) > self.distance_threshold:
                    wv.desiredWV_R = self.k*distance_remaining
                    wv.desiredWV_L = self.k*distance_remaining
                    print "distance_remaining", distance_remaining
                else:
                    wv.desiredWV_R = 0
                    wv.desiredWV_L = 0
                    self.arrived = True
            else:
                wv.desiredWV_R = 0
                wv.desiredWV_L = 0
                if self.stall_count > 200:
                    self.arrived = True
                self.stall_count += 1
        else:
            print "bad data"
            wv.desiredWV_R = 0
            wv.desiredWV_L = 0

        self.velcmd_pub.publish(wv)
        rospy.sleep(0.01)

    def next_input(self):
        if self.tag_id == 0:
            return (2, 220, self.TURN_RIGHT)
        elif self.tag_id == 2:
            return (4, 230, self.TURN_RIGHT)
        elif self.tag_id == 4:
            return 9
        elif self.tag_id == 9:
            return (7, 480, self.TURN_LEFT)
        elif self.tag_id == 7:
            return 8
        else: 
            return (0, 0, 0)

    def next_state(self):
        if self.tag_id == 0 or self.tag_id == 2 or self.tag_id == 9:
            return turn_tag_position.TurnTagPosition(self.next_input())
        elif self.tag_id == 4 or self.tag_id == 7:
            return search.Search(self.next_input())
        else:
            return Stop(self.next_input())

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

    def distance_callback(self, depth_data):
        try:
            cv_depthimage = self.cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
            cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_depthimage, "passthrough"))
        except CvBridgeError as e:
            print(e)
        
        if self.tag_id in self.tags_in_view:
            apriltag_center_x = self.tag_centers[self.tag_id][0]
            apriltag_center_y = self.tag_centers[self.tag_id][1]
            self.apriltag_distance = float(cv_depthimage2[apriltag_center_y][apriltag_center_x])

    def __str__(self):
        return "DriveUntilTagInDistance(%s, %s)" % (self.tag_id, self.desired_dist_away) 