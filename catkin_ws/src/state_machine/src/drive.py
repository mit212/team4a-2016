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
from state import State
from stop import Stop
from release import Release
import search

class Drive(State):
    def __init__(self, current_input):
        self.current_input = current_input

        self.count = 0

        self.arrived = False
        #self.far_obstacles = rospy.get_param("field_has_far_obstacles")
        self.far_obstacles = False

        self.tags_in_view = []
        self.detection_poses = {}
        self.tag_centers = {}
        
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        rospy.sleep(0.5)
        
        self.accum_x = []
        self.accum_y = []
        self.accum_theta = []

        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)
        self.arduino_data_sub = rospy.Subscriber("/arduino_data", ArduinoData, self.arduino_data_callback, queue_size = 1)
        
        if current_input - int(current_input) == 0.5:
            self.far_obstacles = True
            self.current_input = int(current_input)


        ####### Distance detection #######

        # Publisher for converting a CV Image to a ROS Image
        self.image_pub = rospy.Publisher('/distance_image', Image, queue_size = 1)

        # Bridge to convert ROS Image type to OpenCV Image type
        self.cv_bridge = CvBridge()

        depth_sub = rospy.Subscriber("/camera/depth_registered/image", Image, self.distance_callback, queue_size = 1)

        self.apriltag_distance = 20
        ##################################

        ### Point list
        self.point0 = Pose2D(0.32, 1.0, np.pi/2)
        self.point2a = Pose2D(0.32, 1.22, np.pi/2) # y was 1.52
        self.point2b = Pose2D(0.99, 1.36, np.pi/2)
        self.point3 = Pose2D(1.57, 1.55, 4*np.pi/3)
        self.point4 = Pose2D(0.99, 2.13, np.pi)

        self.target_pose_list = self.get_target_pose_list()
        self.current_target = self.target_pose_list[0]
        self.current_target_index = 0

        self.prev_pos_x = []
        self.prev_pos_y = []
        self.prev_yaw = []
        self.filtering_on = True

        self.enc_x = 0
        self.enc_y = 0
        self.enc_theta = 0
        self.hertz = 0
        self.is_safe = 0
        self.wrist_bumper_state = 0

        self.robot_length = 0.53

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        self.saved_theta = 0;

        self.get_current_point()

        self.alpha = 0

        self.x_threshold = 0.05
        self.y_threshold = 0.05
        self.theta_threshold = 0.015

        self.enc_delta_threshold = 0.1

        self.model_tolerance = 0.8

        self.TURN_RIGHT = 1
        self.TURN_LEFT = -self.TURN_RIGHT

        self.turn_directions = {}
        self.turn_directions[3] = self.TURN_LEFT
        self.turn_directions[4] = self.TURN_RIGHT
        self.turn_directions[5] = self.TURN_LEFT # CHANGE BACK TO self.TURN_LEFT
        self.turn_directions[6] = self.TURN_LEFT
        self.turn_directions[9] = self.TURN_LEFT
        self.turn_directions[8] = self.TURN_LEFT

        self.tag_angles = {}
        self.tag_angles[6] = 0
        self.tag_angles[7] = 0
        self.tag_angles[10] = 0
        self.tag_angles[2] = np.pi/2
        self.tag_angles[4] = np.pi/2
        self.tag_angles[5] = np.pi/2
        self.tag_angles[0] = np.pi
        self.tag_angles[1] = np.pi
        self.tag_angles[3] = np.pi
        self.tag_angles[8] = -np.pi/2
        self.tag_angles[9] = -np.pi/2

        #self.turn_to_tag(self.turn_directions[current_input])
        self.turn_to_tag(-self.turn_directions[current_input])
        #for 6, TL
        rospy.sleep(0.5)

    def run(self):
        self.filtering_on = True
        self.get_current_point()
        #print self.current_x, self.current_y, self.current_theta
        self.saved_theta = self.current_theta
        print "apriltag_distance", self.apriltag_distance
        print "tag center", self.tag_centers[self.current_input]
        if self.current_x == 0:
            print "bad data, try again"
        elif len(self.prev_pos_x) == 20:
            self.filtering_on = True
            target_pose = self.get_target_pose_list()[0]
            self.drive_simple(target_pose)
        rospy.sleep(.01)
        
    
    def next_input(self):
        if self.current_input == 0:
            return self.current_input
        elif self.current_input == 5:
            return self.current_input
        elif self.current_input == 6:
            return "pikachu"
        elif self.current_input == 8:
            return "pikachu"
        elif self.current_input == 9:
            return "pidgey"
        return self.current_input

    def next_state(self):
        if self.current_input == 0: # or self.current_input == 6:
            return search.Search(self.next_input())

        elif self.current_input == 5:
            return Release(self.next_input())

        elif self.current_input == 6:
            return PrepareToCatch(self.next_input())

        elif self.current_input == 8:
            return PrepareToCatch(self.next_input())

        elif self.current_input == 9:
            return PrepareToCatch(self.next_input())

        return Stop(self.next_input())

    def is_finished(self):
        return self.arrived

    def is_stop_state(self):
        return False
    
    ## update to include more input options
    def get_target_pose_list(self):
        if self.current_input == 0:
            return [self.point0]

        elif self.current_input == 2:
            if self.far_obstacles:
                return [self.point0, self.point2a]
            #return [self.point0, self.point2b]
            return [self.point0]

        elif self.current_input == 5:
            return [Pose2D(2.44, 1.75, 0.87)]

        elif self.current_input == 6:
            return [Pose2D(2.90, 1.80, 0.64)]

        elif self.current_input == 8:
            return [Pose2D(3.07, 1.1, 0.8)]

        elif self.current_input == 9:
            return [Pose2D(1.98, 1.1, 1)]

        return [Pose2D(0, 0, 0)]

    def apriltag_callback(self, data):
        del self.tags_in_view[:]
        for detection in data.detections:
            corner_x = []
            corner_y = []
            self.tags_in_view.append(detection.id)
            self.detection_poses[detection.id] = detection.pose
            for point in detection.corners2d:
                corner_x.append(point.x)
                corner_y.append(point.y)
            center_x = sum(corner_x) / len(corner_x)
            center_y = sum(corner_y) / len(corner_y)
            self.tag_centers[detection.id] = (center_x, center_y)

    def arduino_data_callback(self, data):
        if len(self.accum_x) >= 3:
            self.accum_x.pop(0)
            self.accum_y.pop(0)
            self.accum_theta.pop(0)

        self.accum_x.append(data.deltaX)
        self.accum_y.append(data.deltaY)    
        self.accum_theta.append(data.deltaTheta)
        self.is_safe = data.isSafe      
        self.wrist_bumper_state = data.wristBumperState

        self.enc_x = sum(self.accum_x)/len(self.accum_x)
        self.enc_y = sum(self.accum_y)/len(self.accum_y)
        self.enc_theta = sum(self.accum_theta)/len(self.accum_theta)

    def distance_callback(self, depth_data):
        try:
            cv_depthimage = self.cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
            cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
            self.image_pub.publish(self.cv_bridge.cv2_to_imgmsg(cv_depthimage, "passthrough"))
        except CvBridgeError as e:
            print(e)
        
        if self.current_input in self.tags_in_view:
            apriltag_center_x = self.tag_centers[self.current_input][0]
            apriltag_center_y = self.tag_centers[self.current_input][1]
            self.apriltag_distance = float(cv_depthimage2[apriltag_center_y][apriltag_center_x])

    def get_current_point(self):
        apriltag_source_frame = '/apriltag' + str(self.current_input)
        if len(self.prev_pos_x) >= 20:
            self.prev_pos_x.pop(0)
            self.prev_pos_y.pop(0)
            self.prev_yaw.pop(0)
        try:
            if self.current_input in self.tags_in_view:
                poselist_tag_cam = helper.pose2poselist(self.detection_poses[self.current_input])
                pose_tag_base = helper.transformPose(pose = poselist_tag_cam,  sourceFrame = '/camera', targetFrame = '/robot_base', lr = self.listener)
                poselist_base_tag = helper.invPoselist(pose_tag_base)
                pose_base_map = helper.transformPose(pose = poselist_base_tag, sourceFrame = apriltag_source_frame, targetFrame = '/map', lr = self.listener)
                helper.pubFrame(self.br, pose = pose_base_map, frame_id = '/robot_base', parent_frame_id = '/map', npub = 1)
                robot_pose3d = helper.lookupTransform(self.listener, '/map', '/robot_base')

                robot_x = robot_pose3d[0]
                robot_y = robot_pose3d[1]
                robot_yaw = tfm.euler_from_quaternion(robot_pose3d[3:7])[2]
                print "filtering", self.filtering_on, "robot_yaw", robot_yaw
                should_append = True
                if (self.current_input in self.turn_directions) and self.filtering_on:
                    if self.turn_directions[self.current_input] == self.TURN_LEFT:
                        if robot_yaw <= self.tag_angles[self.current_input]:
                            should_append = False
                    elif self.turn_directions[self.current_input] == self.TURN_RIGHT:
                        if robot_yaw >= self.tag_angles[self.current_input]:
                            should_append = False

                if should_append:
                    self.prev_pos_x.append(robot_x)
                    self.prev_pos_y.append(robot_y)
                    self.prev_yaw.append(robot_yaw)

                # self.current_x = self.kalman_filter(self.prev_pos_x) #sum(self.prev_pos_x) / len(self.prev_pos_x)
                # self.current_y = self.kalman_filter(self.prev_pos_y) #sum(self.prev_pos_y) / len(self.prev_pos_y)
                # self.current_theta = self.kalman_filter(self.prev_yaw) #sum(self.prev_yaw) / len(self.prev_yaw)

                self.current_x = sum(self.prev_pos_x) / len(self.prev_pos_x)
                self.current_y = sum(self.prev_pos_y) / len(self.prev_pos_y)
                self.current_theta = sum(self.prev_yaw) / len(self.prev_yaw)
        except:
            # print "tag not in view"
            ex_type, ex, tb = sys.exc_info()
            traceback.print_tb(tb)
            self.current_x = -1
            self.current_y = -1
            self.current_theta = -1

        print "get_current_point", self.current_x, self.current_y, self.current_theta

    def stop(self):
        wv = WheelVelCmd()

        if not rospy.is_shutdown():
            print '1. Tag not in view, Stop'
            wv.desiredWV_R = -0.1 #-0.05  # right, left
            wv.desiredWV_L = 0.1 #0.05
            wv.desiredWrist = 0.0
            self.velcmd_pub.publish(wv)

        rospy.sleep(.01)

    def drive_simple(self, target_pose2d):
        #print "self.current_x", self.current_x
        self.turn_alpha(self.current_input)

        if self.tag_angles[self.current_input] == np.pi/2 or self.tag_angles[self.current_input] == -np.pi/2:
            desired_delta_x = abs(self.current_x - target_pose2d.x)
            self.drive_simple_y(desired_delta_x)

        elif self.tag_angles[self.current_input] == np.pi or self.tag_angles[self.current_input] == 0:
            desired_delta_y = abs(self.current_y - target_pose2d.y)
            print "self.current_y", self.current_y, "desired_delta_y", desired_delta_y
            self.drive_simple_x(desired_delta_y)
        else:
            print "kbai"

        desired_distance = target_pose2d.dist 
        self.drive_simple_distance(desired_distance)

        if self.current_input == 6:
            self.turn_to_special_tag_normal(self.TURN_RIGHT, 8)
        elif self.current_input == 9:
            #self.turn_to_tag_normal(self.TURN_RIGHT)
            pass

        self.arrived = True

    def drive_simple_x(self, desired_delta_x):
        wv = WheelVelCmd()
        wv.desiredWrist = 0.0

        current_delta_x = 0
        prev_enc_count = self.enc_x
        start_x = self.current_x
        print "start_x", start_x
        desired_x = start_x + desired_delta_x
        k = .6

        if self.alpha >= 0:
            turn_direction = self.TURN_LEFT
        else:
            turn_direction = self.TURN_RIGHT

        num_tries = 0

        #print "start encoder count", prev_enc_count
        start_enc_count = prev_enc_count
        while abs(desired_x-self.current_x) > self.x_threshold:
            if num_tries > 0:
                print "trying again"
                while True:
                    pass
                self.turn_alpha(self.current_input)

            while not (desired_delta_x - self.x_threshold < current_delta_x < desired_delta_x + self.x_threshold):
                wv.desiredWV_R = k*abs(desired_delta_x-current_delta_x)
                wv.desiredWV_L = k*abs(desired_delta_x-current_delta_x)
                self.velcmd_pub.publish(wv)
                
                x_inc = abs(self.enc_x - prev_enc_count)
                if x_inc <= self.enc_delta_threshold:
                    current_delta_x = abs(self.enc_x - start_enc_count)
                else:
                    print "ignoring"
                
                rospy.sleep(0.01)

                prev_enc_count = self.enc_x
                print "desired_delta_x", desired_delta_x, "current_delta_x:", current_delta_x
                
            wv.desiredWV_R = 0
            wv.desiredWV_L = 0
            self.velcmd_pub.publish(wv)
            rospy.sleep(0.01)

            self.turn_to_tag(turn_direction)
            self.get_current_point()
            print "abs(desired_x-self.current_x)", abs(desired_x-self.current_x)
            # resolve current point
            est_x = start_x + current_delta_x
            obs_x = self.current_x
            if abs(est_x - obs_x) > self.model_tolerance: # they are too far off
                print "resolving"
                #self.get_current_point() # take the apriltag data
                self.current_x = est_x
            else:
                self.current_x = est_x
            print "check try x again? abs(desired_x-self.current_x)", abs(desired_x-self.current_x)
            num_tries += 1

        print "drive_simple_x complete"
        print "took num_tries:", num_tries, "current_x:", self.current_x, "current_alpha", self.current_theta

    def drive_simple_y(self, desired_delta_y):
        wv = WheelVelCmd()
        wv.desiredWrist = 0.0

        current_delta_y = 0
        prev_enc_count = self.enc_y
        start_y = self.current_y
        print "start_y", start_y
        desired_y = start_y + desired_delta_y
        k = .6

        if self.alpha >= 0:
            turn_direction = self.TURN_LEFT
        else:
            turn_direction = self.TURN_RIGHT

        num_tries = 0

        #print "start encoder count", prev_enc_count
        start_enc_count = prev_enc_count
        while abs(desired_y-self.current_y) > self.y_threshold:
            if num_tries > 0:
                print "trying again"
                while True:
                    pass
                self.turn_alpha(self.current_input)

            while not (desired_delta_y - self.y_threshold < current_delta_y < desired_delta_y + self.y_threshold):
                print "self.current_y", self.current_y, "desired_delta_y:", desired_delta_y, "current_delta_y:", current_delta_y
                print "current encoder location", self.enc_y
                wv.desiredWV_R = k*abs(desired_delta_y-current_delta_y)
                wv.desiredWV_L = k*abs(desired_delta_y-current_delta_y)
                self.velcmd_pub.publish(wv)
                
                y_inc = abs(self.enc_y - prev_enc_count)
                if y_inc <= self.enc_delta_threshold:
                    current_delta_y = abs(self.enc_y - start_enc_count)
                else:
                    print "ignoring"
                
                rospy.sleep(0.01)

                prev_enc_count = self.enc_y
                print "current encoder count", prev_enc_count - start_enc_count, "current_delta_y:", current_delta_y
                
            wv.desiredWV_R = 0
            wv.desiredWV_L = 0
            self.velcmd_pub.publish(wv)
            rospy.sleep(0.01)

            self.turn_to_tag(turn_direction)
            self.get_current_point()
            print "abs(desired_y-self.current_y)", abs(desired_y-self.current_y)
            # resolve current point
            est_y = start_y + current_delta_y
            obs_y = self.current_y
            if abs(est_y - obs_y) > self.model_tolerance: # they are too far off
                print "resolving"
                #self.get_current_point() # take the apriltag data
                self.current_y = est_y
            else:
                self.current_y = est_y
            print "check try y again? abs(desired_y-self.current_y)", abs(desired_y-self.current_y)
            num_tries += 1

        print "drive_simple_y complete"
        print "took num_tries:", num_tries, "current_y:", self.current_y, "current_alpha", self.current_theta


    def drive_simple_distance(self, desired_distance):
        #desired_distance += self.robot_length
        print "enter drive_simple_distance"
        wv = WheelVelCmd()
        wv.desiredWrist = 0.0

        k = .8

        distance_remaining = self.apriltag_distance - desired_distance
        distance_threshold = 0.01

        print "desired_distance", desired_distance, "distance_remaining", distance_remaining

        while abs(distance_remaining) > distance_threshold:
            print "desired_distance", desired_distance, "distance_remaining", distance_remaining
            wv.desiredWV_R = k*distance_remaining
            wv.desiredWV_L = k*distance_remaining
            distance_remaining = self.apriltag_distance - desired_distance
            self.velcmd_pub.publish(wv)
            rospy.sleep(0.01)

        wv.desiredWV_R = 0
        wv.desiredWV_L = 0
        self.velcmd_pub.publish(wv)
        rospy.sleep(0.01)

        print "drive_simple_distance complete"

    def turn_alpha(self, target_tag):
        #print self.current_x, self.current_y, self.current_theta
        

        wv = WheelVelCmd()
        wv.desiredWrist = 0.0
        #self.filtering_on = True

        offset = self.tag_angles[target_tag]
        self.alpha = -(self.current_theta - offset)

        if self.current_input in self.turn_directions:
            turn_direction = self.turn_directions[self.current_input]
        elif self.alpha <= 0:
            turn_direction = self.TURN_LEFT
        else:
            turn_direction = self.TURN_RIGHT

        #print "alpha", self.alpha, "turn_direction", turn_direction

        current_delta_alpha = 0 #how much alpha we have gone, pos if TL/neg if TR
        start_enc_theta = self.enc_theta #what is enc theta right now, before moving?
        start_enc_x = self.enc_x
        start_enc_y = self.enc_y

        if turn_direction == self.TURN_LEFT:
            turn_angle = np.pi/2 - (-self.alpha)
        elif turn_direction == self.TURN_RIGHT:
            turn_angle = self.alpha - np.pi/2

        while not (turn_angle - self.theta_threshold < current_delta_alpha < turn_angle + self.theta_threshold):
            #print "self.alpha:", self.alpha, "turn_angle", turn_angle, "current_delta_alpha:", current_delta_alpha
            #print "start_enc_theta", start_enc_theta, "self.enc_theta", self.enc_theta
            if turn_direction == self.TURN_LEFT:
                wv.desiredWV_R = 0.1
                wv.desiredWV_L = -0.1
            elif turn_direction == self.TURN_RIGHT:
                wv.desiredWV_R = -0.1
                wv.desiredWV_L = 0.1
            else:
                print "kbai"

            self.velcmd_pub.publish(wv)
            current_delta_alpha = self.enc_theta-start_enc_theta
            rospy.sleep(0.01)

        wv.desiredWV_R = 0
        wv.desiredWV_L = 0
        self.velcmd_pub.publish(wv)
        rospy.sleep(0.01)

        print "before end of turn_alpha", self.current_x
        self.current_x = self.current_x + (self.enc_x - start_enc_x)
        self.current_y = self.current_y + (self.enc_y - start_enc_y)
        print "end of turn_alpha", self.current_x
        #self.filtering_on = False
        print "turn_alpha complete"

    def turn_to_tag(self, direction):
        wv = WheelVelCmd()

        while self.current_input not in self.tags_in_view:
            if direction == self.TURN_RIGHT:
                wv.desiredWV_R = -0.1
                wv.desiredWV_L = 0.1
            elif direction == self.TURN_LEFT:
                wv.desiredWV_R = 0.1
                wv.desiredWV_L = -0.1
            else:
                print "invalid direction"
            wv.desiredWrist = 0.0
            self.velcmd_pub.publish(wv)
            rospy.sleep(0.01)

        camera_center_x = 320
        camera_x_threshold = 5
        tag_x = self.tag_centers[self.current_input][0]
        while not camera_center_x - camera_x_threshold < tag_x < camera_center_x + camera_x_threshold:
            print "tag_x", tag_x
            if direction == self.TURN_RIGHT:
                wv.desiredWV_R = -0.1
                wv.desiredWV_L = 0.1
            elif direction == self.TURN_LEFT:
                wv.desiredWV_R = 0.1
                wv.desiredWV_L = -0.1
            else:
                print "invalid direction"
            self.velcmd_pub.publish(wv)    
            rospy.sleep(0.01)
            tag_x = self.tag_centers[self.current_input][0]

        wv.desiredWV_R = 0
        wv.desiredWV_L = 0
        wv.desiredWrist = 0.0
        self.velcmd_pub.publish(wv)
        rospy.sleep(0.01)

    def turn_to_tag_normal(self, direction):
        wv = WheelVelCmd()
        wv.desiredWrist = 0.0

        while self.current_input not in self.tags_in_view:
            if direction == self.TURN_RIGHT:
                wv.desiredWV_R = -0.1
                wv.desiredWV_L = 0.1
            elif direction == self.TURN_LEFT:
                wv.desiredWV_R = 0.1
                wv.desiredWV_L = -0.1
            else:
                print "invalid direction"
            
            self.velcmd_pub.publish(wv)
            rospy.sleep(0.01)

        camera_center_x = 320
        camera_x_threshold = 5
        tag_x = self.tag_centers[self.current_input][0]
        while not camera_center_x - camera_x_threshold < tag_x < camera_center_x + camera_x_threshold:
            if tag_x <= camera_center_x - camera_x_threshold:
                wv.desiredWV_R = -0.1
                wv.desiredWV_L = 0.1
            elif tag_x >= camera_center_x + camera_x_threshold:
                wv.desiredWV_R = 0.1
                wv.desiredWV_L = -0.1
            else:
                print "invalid direction"
            self.velcmd_pub.publish(wv)    
            rospy.sleep(0.01)

        wv.desiredWV_R = 0
        wv.desiredWV_L = 0
        self.velcmd_pub.publish(wv)
        rospy.sleep(0.01)

    def turn_to_special_tag_normal(self, direction, tag):
        wv = WheelVelCmd()
        wv.desiredWrist = 0.0

        while tag not in self.tags_in_view:
            if direction == self.TURN_RIGHT:
                wv.desiredWV_R = -0.1
                wv.desiredWV_L = 0.1
            elif direction == self.TURN_LEFT:
                wv.desiredWV_R = 0.1
                wv.desiredWV_L = -0.1
            else:
                print "invalid direction"
            
            self.velcmd_pub.publish(wv)
            rospy.sleep(0.01)

        camera_center_x = 320
        camera_x_threshold = 5
        tag_x = self.tag_centers[tag][0]
        while not camera_center_x - camera_x_threshold < tag_x < camera_center_x + camera_x_threshold:
            if direction == self.TURN_RIGHT:
                wv.desiredWV_R = -0.1
                wv.desiredWV_L = 0.1
            elif direction == self.TURN_LEFT:
                wv.desiredWV_R = 0.1
                wv.desiredWV_L = -0.1
            else:
                print "invalid direction"
            self.velcmd_pub.publish(wv)    
            rospy.sleep(0.01)

        wv.desiredWV_R = 0
        wv.desiredWV_L = 0
        self.velcmd_pub.publish(wv)
        rospy.sleep(0.01)

    def drive(self, target_pose2d):
        wv = WheelVelCmd()

        print target_pose2d

        if not rospy.is_shutdown():
            
            # 1. get robot pose
            robot_pose3d = helper.lookupTransform(self.listener, '/map', '/robot_base')
            
            # this should never happen
            if robot_pose3d is None:
                print '1. Tag not in view, Stop'
                wv.desiredWV_R = 0.0  # right, left
                wv.desiredWV_L = 0.0
                self.velcmd_pub.publish(wv)  
                rospy.sleep(0.01)
                return
            
            if len(self.prev_pos_x) < 10:
                self.prev_pos_x.append(robot_pose3d[0])
            else:
                self.prev_pos_x = self.prev_pos_x[1:] + [robot_pose3d[0]]

            if len(self.prev_pos_y) < 10:
                self.prev_pos_y.append(robot_pose3d[1])
            else:
                self.prev_pos_y = self.prev_pos_y[1:] + [robot_pose3d[1]]

            print self.prev_pos_x
            print self.prev_pos_y

            robot_position2d = [self.kalman_filter(self.prev_pos_x), self.kalman_filter(self.prev_pos_y)]
            target_position2d = target_pose2d.pose_list[0:2]
            
            robot_yaw = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
            robot_pose2d = robot_position2d + [robot_yaw]
            
            if len(self.prev_yaw) < 3:
                self.prev_yaw.append(robot_yaw)
            else:
                self.prev_yaw = self.prev_yaw[1:] + [robot_yaw]

            robot_yaw = sum(self.prev_yaw) / len(self.prev_yaw)
            # 2. navigation policy
            # 2.1 if       in the target, 
            # 2.2 else if  close to target position, turn to the target orientation
            # 2.3 else if  in the correct heading, go straight to the target position,
            # 2.4 else     turn in the direction of the target position
            
            pos_delta = np.array(target_position2d) - np.array(robot_position2d)
            robot_heading_vec = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
            heading_err_cross = helper.cross2d( robot_heading_vec, pos_delta / np.linalg.norm(pos_delta) )
            
            pos_delta_unit_vec = pos_delta / np.linalg.norm(pos_delta)

            robot_target_angle = np.arccos(np.clip(np.dot(robot_heading_vec, pos_delta_unit_vec), -1.0, 1.0))
            robot_orientation_angle = helper.diffrad(robot_yaw, target_pose2d.theta)

            print 'robot_position2d', robot_position2d, 'target_position2d', target_position2d
            print 'pos_delta', pos_delta
            print 'robot_yaw', robot_yaw
            print 'norm delta', np.linalg.norm( pos_delta ), 'diffrad', np.fabs(helper.diffrad(robot_yaw, target_pose2d.theta))
            print 'heading_err_cross', heading_err_cross
            print 'robot_target_angle', robot_target_angle
            
            distance_error = 0.08
            angle_error = 0.05

            k_straight =  0.2
            k_turn_target = 0.3
            k_turn_orientation = 0.1

            max_speed = 0.1

            if target_pose2d.arrived or np.linalg.norm(pos_delta) < distance_error and np.fabs(robot_orientation_angle) < angle_error:
                print 'Arrived at target, stopping'
                wv.desiredWV_R = 0  
                wv.desiredWV_L = 0
                target_pose2d.arrived = True
            elif not target_pose2d.aligned and not target_pose2d.arrived_position:
                print 'Turning towards target'
                mult = heading_err_cross / np.fabs(heading_err_cross)
                wv.desiredWV_R = k_turn_target * robot_target_angle * mult
                wv.desiredWV_L = -k_turn_target * robot_target_angle * mult
                if np.fabs(robot_target_angle) < angle_error:
                    target_pose2d.aligned = True
            elif not target_pose2d.arrived_position and np.linalg.norm(pos_delta) > distance_error:
                print 'Driving towards target'
                wv.desiredWV_R = min(k_straight * np.linalg.norm(pos_delta), max_speed)
                wv.desiredWV_L = min(k_straight * np.linalg.norm(pos_delta), max_speed)
            else:
                print 'Turning towards orientation'
                mult = robot_orientation_angle / np.fabs(robot_orientation_angle)
                wv.desiredWV_R = -k_turn_orientation * robot_orientation_angle * mult
                wv.desiredWV_L = k_turn_orientation * robot_orientation_angle * mult
                target_pose2d.arrived_position = True

            print
            # TODO: replace with real controller

            '''
            k = np.linalg.norm(pos_delta)
            print k
            # print "k", k
            # print "other", np.fabs(helper.diffrad(robot_yaw, target_pose2d.theta))
            # general strategy:
            # turn towards target point
            # drive straight to target point
            # turn towards final orientation
            if target_pose2d.arrived or k < 0.4 and np.fabs(helper.diffrad(robot_yaw, target_pose2d.theta)) < 0.05:
                print 'Case 2.1 Stop'
                wv.desiredWV_R = 0  
                wv.desiredWV_L = 0
                target_pose2d.arrived = True
            elif not target_pose2d.arrived_position or np.fabs( heading_err_cross ) > 0.3:
                print 'Case 2.2 Straight forward'
                wv.desiredWV_R = 0.1 * k
                wv.desiredWV_L = 0.1 * k
            else:
                print 'Case 2.1 Turning'
                if k < 0.4:
                    target_pose2d.arrived_position = True
                print k
                mult = heading_err_cross / np.fabs(heading_err_cross)
                wv.desiredWV_R = 0.1 * k * mult * 0.5
                wv.desiredWV_L = -0.1 * k * mult * 0.5
            ''' 

            wv.desiredWrist = 0.0
            self.velcmd_pub.publish(wv)
            
            self.count += 1
            rospy.sleep(0.02)

    def kalman_filter(self, z):
        # intial parameters
        n_iter = len(z)
        sz = (n_iter,)

        Q = 1e-5 # process variance

        # allocate space for arrays
        xhat=np.zeros(sz)      # a posteri estimate of x
        P=np.zeros(sz)         # a posteri error estimate
        xhatminus=np.zeros(sz) # a priori estimate of x
        Pminus=np.zeros(sz)    # a priori error estimate
        K=np.zeros(sz)         # gain or blending factor

        R = 0.15**2 # estimate of measurement variance, change to see effect

        # intial guesses
        xhat[0] = z[0]
        P[0] = z[0] + 0.1 # estimate we're 10 cm away from our initial guess

        for k in range(1,n_iter):
            # time update
            xhatminus[k] = xhat[k-1]
            Pminus[k] = P[k-1]+Q

            # measurement update
            K[k] = Pminus[k]/( Pminus[k]+R )
            xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k])
            P[k] = K[k]*Pminus[k]

        return Pminus[-1]

    def __str__(self):
        return "Drive(%s)" % (self.current_input)

class Pose2D():
    def __init__(self, x, y, dist):
        self.x = x
        self.y = y
        self.dist = dist

        self.pose_list = [x, y, dist]

        self.arrived = False
        self.aligned = False
        self.arrived_position = False

    def __str__(self):
        return "Pose2d: %s %s %s" % (self.x, self.y, self.dist)