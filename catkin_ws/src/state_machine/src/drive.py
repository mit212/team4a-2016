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

class Drive(State):
    def __init__(self, current_input):
        self.current_input = current_input
        self.arrived = False
        self.tags_in_view = []
        self.detection_pose = None
        
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)
        
        #self.thread = threading.Thread(target = self.drive)
        #self.thread.start()
        #rospy.spin()
        
    def run(self):
        if self.current_input in self.tags_in_view:
            #print "driving to apriltag", self.current_input
            #print helper.invPoselist
            pose_tag_base = helper.transformPose(pose = helper.pose2poselist(self.detection_pose),  sourceFrame = '/camera', targetFrame = '/base_link', lr = self.listener)
            #print "pose_tag_base", pose_tag_base
            pose_base_map = helper.transformPose(pose = helper.invPoselist(pose_tag_base), sourceFrame = '/apriltag', targetFrame = '/map', lr = self.listener)
            helper.pubFrame(self.br, pose = pose_base_map, frame_id = '/base_link', parent_frame_id = '/map', npub = 1)
        self.drive()
        #rospy.spin()
    
    def next_input(self):
        return 0 # change later

    def next_state(self):
        return Stop(self.next_input())

    def is_finished(self):
        return self.arrived

    def is_stop_state(self):
        return Falseself.apriltag_callback
        
    def apriltag_callback(self, data):
        del self.tags_in_view[:]
        for detection in data.detections:
            #print detection.pose.position.x, detection.pose.position.y, detection.pose.position.z
            self.tags_in_view.append(detection.id)
            self.detection_pose = detection.pose
    
    # probably put this code in the run method, this just here for testing
    def drive(self):
        #print "driving"
        ##
        target_pose2d = [0.25, 0, np.pi]
        
        ##
        wv = WheelVelCmd()
        
        ##
        arrived_position = False
        
        #change to if
        while not rospy.is_shutdown() :
            
            ## 
            # 1. get robot pose
            robot_pose3d = helper.lookupTransform(self.listener, '/map', '/base_link')
            
            if robot_pose3d is None:
                print '1. Tag not in view, Stop'
                wv.desiredWV_R = 0  # right, left
                wv.desiredWV_L = 0
                self.velcmd_pub.publish(wv)  
                continue
            
            robot_position2d = robot_pose3d[0:2]
            target_position2d = target_pose2d[0:2]
            
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
            # print 'norm delta', np.linalg.norm( pos_delta ), 'diffrad', diffrad(robot_yaw, target_pose2d[2])
            # print 'heading_err_cross', heading_err_cross
            
            if self.arrived or (np.linalg.norm( pos_delta ) < 0.08 and np.fabs(diffrad(robot_yaw, target_pose2d[2]))<0.05) :
                print 'Case 2.1  Stop'
                wv.desiredWV_R = 0  
                wv.desiredWV_L = 0
                self.arrived = True
            elif np.linalg.norm( pos_delta ) < 0.08:
                arrived_position = True
                if diffrad(robot_yaw, target_pose2d[2]) > 0:
                    print 'Case 2.2.1  Turn right slowly'      
                    wv.desiredWV_R = -0.05 
                    wv.desiredWV_L = 0.05
                else:
                    print 'Case 2.2.2  Turn left slowly'
                    wv.desiredWV_R = 0.05  
                    wv.desiredWV_L = -0.05
                    
            elif arrived_position or np.fabs( heading_err_cross ) < 0.2:
                print 'Case 2.3  Straight forward'  
                wv.desiredWV_R = 0.1
                wv.desiredWV_L = 0.1
            else:
                if heading_err_cross < 0:
                    print 'Case 2.4.1  Turn right'
                    wv.desiredWV_R = -0.1
                    wv.desiredWV_L = 0.1
                else:
                    print 'Case 2.4.2  Turn left'
                    wv.desiredWV_R = 0.1
                    wv.desiredWV_L = -0.1
                    
            self.velcmd_pub.publish(wv)  
            
            rospy.sleep(0.01)

