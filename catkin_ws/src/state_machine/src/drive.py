import rospy
import tf
import numpy as np
import threading
import serial
import pdb
import traceback
import sys
import tf.transformations as tfm

from me212cv.msg import DetectedObject
from me212base.msg import WheelVelCmd
from apriltags.msg import AprilTagDetections
import me212helper.helper as helper

from state import State
from stop import Stop

class Drive(State):
    def __init__(self, current_input):
        self.current_input = current_input

        self.arrived = False

        self.target_pose_list = self.get_target_pose_list()
        self.current_target = self.target_pose_list[0]
        self.current_target_index = 0

        self.tags_in_view = []
        self.detection_poses = {}
        
        self.obstacles = []

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        rospy.sleep(0.5)
        
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)
        
        self.object_sub = rospy.Subscriber("/object_info", DetectedObject, self.obstacle_callback, queue_size = 1)
        rospy.sleep(1)
        self.far_obstacles = self.determine_obstacles()
        
    def run(self):
        apriltag_source_frame = '/apriltag' + str(self.current_input)

        print self.far_obstacles

        if self.current_input in self.tags_in_view:
            poselist_tag_cam = helper.pose2poselist(self.detection_poses[self.current_input])
            pose_tag_base = helper.transformPose(pose = poselist_tag_cam,  sourceFrame = '/camera', targetFrame = '/robot_base', lr = self.listener)
            poselist_base_tag = helper.invPoselist(pose_tag_base)
            pose_base_map = helper.transformPose(pose = poselist_base_tag, sourceFrame = apriltag_source_frame, targetFrame = '/map', lr = self.listener)
            helper.pubFrame(self.br, pose = pose_base_map, frame_id = '/robot_base', parent_frame_id = '/map', npub = 1)
            #print self.current_target
            #print "before", self.current_target.arrived
            #self.drive(self.current_target)
            #print "after", self.current_target.arrived
            if self.current_target.arrived:
                if self.current_target_index == len(self.target_pose_list) - 1:
                    self.arrived = True
                    return
                self.current_target_index += 1
                self.current_target = self.target_pose_list[self.current_target_index]
        else:
            self.stop()
    
    def next_input(self):
        return 0 # change later

    def next_state(self):
        return Stop(self.next_input())

    def is_finished(self):
        return self.arrived

    def is_stop_state(self):
        return False
    
    ## change to return list and update boolean logic
    ## next steps: use object detection to get next pose
    def get_target_pose_list(self):
        if self.current_input == 2:
            return [Pose2D(.25, 0.1, np.pi/2), Pose2D(0.25, 0.9, np.pi/2)]
        return [Pose2D(0, 0, 0)]

    def apriltag_callback(self, data):
        del self.tags_in_view[:]
        for detection in data.detections:
            self.tags_in_view.append(detection.id)
            self.detection_poses[detection.id] = detection.pose

    def obstacle_callback(self,data):
        # want to add datapoints with significant width/height
        if data.width > 40 and data.height > 40:
            self.obstacles.append(data)
    
    # probably add in distance too
    def determine_obstacles(self):
        relevant_obstacles = self.obstacles[-5:]
        far_obs_count = 0
        near_obs_count = 0
        print relevant_obstacles
        for obstacle in relevant_obstacles:
            if obstacle.center_x >= 165 and obstacle.center_x <= 200 and obstacle.center_y >= 360 and obstacle.center_y <= 390:
                if obstacle.width >= 350 and obstacle.height >= 170:
                    return True
        return False


    def stop(self):
        wv = WheelVelCmd()

        if not rospy.is_shutdown():
            print '1. Tag not in view, Stop'
            wv.desiredWV_R = 0  # right, left
            wv.desiredWV_L = 0
            self.velcmd_pub.publish(wv)

        rospy.sleep(.01)

    def drive(self, target_pose2d):
        wv = WheelVelCmd()

        print target_pose2d

        if not rospy.is_shutdown():
            
            # 1. get robot pose
            robot_pose3d = helper.lookupTransform(self.listener, '/map', '/robot_base')
            
            if robot_pose3d is None:
                print '1. Tag not in view, Stop'
                wv.desiredWV_R = 0  # right, left
                wv.desiredWV_L = 0
                self.velcmd_pub.publish(wv)  
                return
            
            robot_position2d = robot_pose3d[0:2]
            target_position2d = target_pose2d.pose_list[0:2]
            
            robot_yaw = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
            robot_pose2d = robot_position2d + [robot_yaw]
            
            # 2. navigation policy
            # 2.1 if       in the target, 
            # 2.2 else if  close to target position, turn to the target orientation
            # 2.3 else if  in the correct heading, go straight to the target position,
            # 2.4 else     turn in the direction of the target position
            
            pos_delta = np.array(target_position2d) - np.array(robot_position2d)
            robot_heading_vec = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
            heading_err_cross = helper.cross2d( robot_heading_vec, pos_delta / np.linalg.norm(pos_delta) )
            
            # print 'robot_position2d', robot_position2d, 'target_position2d', target_position2d
            # print 'pos_delta', pos_delta
            # print 'robot_yaw', robot_yaw
            # print 'norm delta', np.linalg.norm( pos_delta ), 'diffrad', diffrad(robot_yaw, target_pose2d.theta)
            # print 'heading_err_cross', heading_err_cross

            # TODO: clean up all these magic numbers

            # TODO: replace with real controller

            k = np.linalg.norm(pos_delta)
            print k
            # print "k", k
            # print "other", np.fabs(helper.diffrad(robot_yaw, target_pose2d.theta))

            if target_pose2d.arrived or k < 0.4 and np.fabs(helper.diffrad(robot_yaw, target_pose2d.theta)) < 0.2:
                print 'Case 2.1 Stop'
                wv.desiredWV_R = 0  
                wv.desiredWV_L = 0
                target_pose2d.arrived = True
            elif target_pose2d.arrived_position or np.fabs( heading_err_cross ) < 0.3:
                print 'Case 2.2 Straight forward'
                wv.desiredWV_R = 0.1 * k * 1.5
                wv.desiredWV_L = 0.1 * k * 1.5
            else:
                print 'Case 2.1 Turning'
                if k < 0.4:
                    target_pose2d.arrived_position = True
                print k
                mult = heading_err_cross / np.fabs(heading_err_cross)
                wv.desiredWV_R = 0.1 * k * mult
                wv.desiredWV_L = -0.1 * k * mult
                    
            self.velcmd_pub.publish(wv)  
            
            rospy.sleep(.01)

class Pose2D():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

        self.pose_list = [x, y, theta]

        self.arrived = False
        self.arrived_position = False

    def __str__(self):
        return "Pose2d: %s %s %s" % (self.x, self.y, self.theta)