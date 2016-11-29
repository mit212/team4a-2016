import rospy
import numpy as np
import message_filters
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Bool, ColorRGBA
from me212cv.msg import DetectedObject
from sensor_msgs.msg import Image, CameraInfo

from state import State
import search

class DetectObstacles(State):
    def __init__(self, current_input):
        self.current_input = current_input
        self.obstacles = []
        self.classified_obstacles = False

        # Publisher for converting a CV Image to a ROS Image
		self.image_pub = rospy.Publisher('/distance_image', Image, queue_size = 1)

		# Publisher for publishing information about the detected objects
		self.object_pub = rospy.Publisher('/object_info', DetectedObject, queue_size = 1)

		# Bridge to convert ROS Image type to OpenCV Image type
		self.cv_bridge = CvBridge()

		image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
        depth_sub = message_filters.Subscriber("/camera/depth_registered/image", Image)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 10, 0.5)
        ts.registerCallback(rosDistCallBack)

        object_sub = rospy.Subscriber("/object_info", DetectedObject, self.obstacle_callback, queue_size = 1)

        rospy.sleep(3)

    def run(self):
        field_has_far_obstacles = self.determine_obstacles()
        rospy.set_param("field_has_far_obstacles", field_has_far_obstacles)
        print "far obstacles:", field_has_far_obstacles

        self.classified_obstacles = True

    def next_input(self):
        return 0

    def next_state(self):
        return search.Search(self.next_input())

    def is_finished(self):
        return self.classified_obstacles

    def is_stop_state(self):
        return False

    def obstacle_callback(self,data):
        # want to add datapoints with significant width/height
        if data.width >= 75 and data.height >= 75:
            self.obstacles.append(data)
    
    def determine_obstacles(self):
        relevant_obstacles = self.obstacles[-5:]
        far_obs_count = 0
        near_obs_count = 0
        for obstacle in relevant_obstacles:
            if obstacle.center_x >= 165 and obstacle.center_x <= 200 and obstacle.center_y >= 360 and obstacle.center_y <= 390:
                if obstacle.width >= 350 and obstacle.height >= 170:
                    return True
        return False

	def rosDistCallBack(self, rgb_data, depth_data):
	    try:
	        cv_image = cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
	        cv_depthimage = cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
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
	            
	        print 'z', center_z

	        obj = DetectedObject()
	        obj.center_x = center_x
	        obj.center_y = center_y
	        obj.center_z = center_z
	        obj.width = w
	        obj.height = h

	        self.object_pub.publish(obj)

	def distanceObjectDetection(cv_depthimage):
	    # range of acceptable distances
	    lower_dist = 0.3
	    upper_dist = 2.0

	    # Threshold the image to only include objects within the specified distance
	    mask = cv2.inRange(cv_depthimage, lower_dist, upper_dist)
	    mask_eroded = cv2.erode(mask, None, iterations = 3)
	    mask_eroded_dilated = cv2.dilate(mask_eroded, None, iterations = 15)
	    
	    try:
	        self.image_pub.publish(cv_bridge.cv2_to_imgmsg(mask_eroded_dilated, "passthrough"))
	    except CvBridgeError as e:
	        print(e)

	    _, contours,hierarchy = cv2.findContours(mask_eroded_dilated,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
	    return contours, mask_eroded_dilated

    def __str__(self):
        return "DetectObstacles(%s)" % (self.current_input)