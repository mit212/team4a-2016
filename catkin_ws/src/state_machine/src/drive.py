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
import search

class Drive(State):
    def __init__(self, current_input):
        global field_has_far_obstacles

        self.current_input = current_input

        self.arrived = False
        #self.far_obstacles = rospy.get_param("field_has_far_obstacles")
        self.far_obstacles = False

        self.tags_in_view = []
        self.detection_poses = {}
        
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        rospy.sleep(0.5)
        
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)
        
        if current_input - int(current_input) == 0.5:
            self.far_obstacles = True
            self.current_input = int(current_input)

        self.target_pose_list = self.get_target_pose_list()
        self.current_target = self.target_pose_list[0]
        self.current_target_index = 0

        self.prev_pos_x = []
        self.prev_pos_y = []
        
    def run(self):
        apriltag_source_frame = '/apriltag' + str(self.current_input)

        if self.current_input in self.tags_in_view:
            poselist_tag_cam = helper.pose2poselist(self.detection_poses[self.current_input])
            pose_tag_base = helper.transformPose(pose = poselist_tag_cam,  sourceFrame = '/camera', targetFrame = '/robot_base', lr = self.listener)
            poselist_base_tag = helper.invPoselist(pose_tag_base)
            pose_base_map = helper.transformPose(pose = poselist_base_tag, sourceFrame = apriltag_source_frame, targetFrame = '/map', lr = self.listener)
            helper.pubFrame(self.br, pose = pose_base_map, frame_id = '/robot_base', parent_frame_id = '/map', npub = 1)
            self.drive(self.current_target)
            if self.current_target.arrived:
                if self.current_target_index == len(self.target_pose_list) - 1:
                    self.arrived = True
                    return
                self.current_target_index += 1
                self.current_target = self.target_pose_list[self.current_target_index]
        else:
            self.stop()
    
    def next_input(self):
        #return 2 # change later
        return 8

    def next_state(self):
        if self.current_input == 0: # or self.current_input == 6:
            return search.Search(self.next_input())
        return Stop(self.next_input())

    def is_finished(self):
        return self.arrived

    def is_stop_state(self):
        return False
    
    ## update to include more input options
    def get_target_pose_list(self):
        if self.current_input == 0:
            return [Pose2D(.15, 0.3, np.pi/2)]
        elif self.current_input == 2:
            if self.far_obstacles:
                return [Pose2D(0.15, 0.3, np.pi/2), Pose2D(0.75, 0.9, 0)]
            return [Pose2D(.15, 0.3, np.pi/2), Pose2D(0.25, 1.5, np.pi/2)]
        elif self.current_input == 6:
            return [Pose2D(3.05, 1.7, 0)]
        elif self.current_input == 8:
            return [Pose2D(3.05, 1.1, -np.pi/2)]
        return [Pose2D(0, 0, 0)]

    def apriltag_callback(self, data):
        del self.tags_in_view[:]
        for detection in data.detections:
            self.tags_in_view.append(detection.id)
            self.detection_poses[detection.id] = detection.pose

    def stop(self):
        wv = WheelVelCmd()

        if not rospy.is_shutdown():
            print '1. Tag not in view, Stop'
            wv.desiredWV_R = 0.0 #-0.05  # right, left
            wv.desiredWV_L = 0.0 #0.05
            self.velcmd_pub.publish(wv)

        rospy.sleep(.01)

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
                return
            
            if len(self.prev_pos_x) < 3:
                self.prev_pos_x.append(robot_pose3d[0])
            else:
                self.prev_pos_x = self.prev_pos_x[1:] + [robot_pose3d[0]]

            if len(self.prev_pos_y) < 3:
                self.prev_pos_y.append(robot_pose3d[1])
            else:
                self.prev_pos_y = self.prev_pos_y[1:] + [robot_pose3d[1]]

            print self.prev_pos_x
            print self.prev_pos_y

            robot_position2d = [sum(self.prev_pos_x) / len(self.prev_pos_x), sum(self.prev_pos_y) / len(self.prev_pos_y)]
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
            angle_error = 0.1

            k_straight =  np.linalg.norm(pos_delta)
            k_turn_target = robot_target_angle
            k_turn_orientation = robot_orientation_angle


            if target_pose2d.arrived or np.linalg.norm(pos_delta) < distance_error and np.fabs(robot_orientation_angle) < angle_error:
                print 'Arrived at target, stopping'
                wv.desiredWV_R = 0  
                wv.desiredWV_L = 0
                target_pose2d.arrived = True
            elif not target_pose2d.arrived_position and np.fabs(robot_target_angle) > angle_error:
                print 'Turning towards target'
                mult = heading_err_cross / np.fabs(heading_err_cross)
                wv.desiredWV_R = 0.35 * k_turn_target * mult
                wv.desiredWV_L = -0.35 * k_turn_target * mult
            elif not target_pose2d.arrived_position and np.linalg.norm(pos_delta) > distance_error:
                print 'Driving towards target'
                wv.desiredWV_R = min(0.3 * k_straight, 0.12)
                wv.desiredWV_L = min(0.3 * k_straight, 0.12)
            else:
                print 'Turning towards orientation'
                mult = robot_orientation_angle / np.fabs(robot_orientation_angle)
                wv.desiredWV_R = -0.1 * k_turn_orientation * mult
                wv.desiredWV_L = 0.1 * k_turn_orientation * mult
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

            self.velcmd_pub.publish(wv)
            
            rospy.sleep(0.15)

    def __str__(self):
        return "Drive(%s)" % (self.current_input)

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