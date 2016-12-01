import rospy
import numpy as np
import message_filters
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

from me212base.msg import WheelVelCmd
from std_msgs.msg import Bool
from me212cv.msg import DetectedObject
from sensor_msgs.msg import Image, CameraInfo

from state import State
from stop import Stop
from catch import Catch

class PrepareToCatch(State):
    def __init__(self, current_input):
        self.current_input = current_input
        self.pokemon = []
        
        self.pikachu = False
        if current_input == "pikachu":
            self.pikachu = True

        self.found_pokemon = False
        self.should_stop = False

        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)

        # Publisher for converting a CV Image to a ROS Image
        self.image_pub = rospy.Publisher('/distance_image', Image, queue_size = 1)

        # Publisher for publishing information about the detected objects
        self.object_pub = rospy.Publisher('/object_info', DetectedObject, queue_size = 1)

        # Bridge to convert ROS Image type to OpenCV Image type
        self.cv_bridge = CvBridge()

        image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
        depth_sub = message_filters.Subscriber("/camera/depth_registered/image", Image)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
        ts.registerCallback(self.rosDistCallBack)

        object_sub = rospy.Subscriber("/object_info", DetectedObject, self.pokemon_callback, queue_size = 1)

    def run(self):
        wv = WheelVelCmd()

        if not self.found_pokemon:
            print "driving"
            wv.desiredWV_R = 0.05
            wv.desiredWV_L = 0.05
        else:
            print "stopping"
            wv.desiredWV_R = 0.0
            wv.desiredWV_L = 0.0
            self.should_stop = True

        self.velcmd_pub.publish(wv)
        rospy.sleep(0.1)

    def next_input(self):
        return 0

    def next_state(self):
        return Catch(self.next_input())

    def is_finished(self):
        return self.should_stop

    def is_stop_state(self):
        return False

    def pokemon_callback(self,data):
        if self.pikachu:
            if data.width >= 40 and data.width <= 70 and data.height >= 40 and data.height <= 70:
                if data.center_x > 300 and data.center_x < 400 and data.center_y > 350 and data.center_y < 450:
                    self.found_pokemon = True
        else:
            if data.width >= 60 and data.height >= 75:
                if data.center_y > 380 and data.center_y < 400:
                    self.found_pokemon = True

    def rosDistCallBack(self, rgb_data, depth_data):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            cv_depthimage = self.cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
            cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
        except CvBridgeError as e:
            print(e)
                
        contours, mask_image = self.distanceObjectDetection(cv_depthimage)

        for cnt in contours:
            xp,yp,w,h = cv2.boundingRect(cnt)
            center_x, center_y = xp+w/2, yp+h/2

            center_z = cv_depthimage2[int(center_y)][int(center_x)]

            # make sure the depth is in the normal range 0.1-10 meter
            if math.isnan(center_z) or center_z < 0.1 or center_z > 10.0:
                continue
                
            obj = DetectedObject()
            obj.center_x = center_x
            obj.center_y = center_y
            obj.center_z = center_z
            obj.width = w
            obj.height = h

            self.object_pub.publish(obj)

    def distanceObjectDetection(self, cv_depthimage):
        # range of acceptable distances
        if self.pikachu:
            lower_dist = 0.52
            upper_dist = 0.57
        else:
            lower_dist = 0.50
            upper_dist = 0.55

        # Threshold the image to only include objects within the specified distance
        mask = cv2.inRange(cv_depthimage, lower_dist, upper_dist)
        mask_eroded = cv2.erode(mask, None, iterations = 3)
        mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 15)
        
        try:
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(mask_eroded_dilated, "passthrough"))
        except CvBridgeError as e:
            print(e)

        _, contours,hierarchy = cv2.findContours(mask_eroded_dilated,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        return contours, mask_eroded_dilated

    def __str__(self):
        return "PrepareToCatch(%s)" % (self.current_input)