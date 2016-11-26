import rospy
import tf
import numpy as np
import threading
import serial
import pdb
import traceback
import sys
import tf.transformations as tfm

from std_msgs.msg import Bool
from me212cv.msg import DetectedObject
import me212helper.helper as helper

from state import State
from stop import Stop

class DetectObstacles(State):
    def __init__(self, current_input):
        self.current_input = current_input
        
        self.obstacles = []
        
        self.object_sub = rospy.Subscriber("/object_info", DetectedObject, self.obstacle_callback, queue_size = 1)
        rospy.sleep(1)
        self.far_obstacles = self.determine_obstacles()

        print self.far_obstacles
        
    def run(self):
        pass

    def next_input(self):
        if self.far_obstacles:
            return 2.5
        return 2 # change later ?

    def next_state(self):
        return Drive(self.next_input())

    def is_finished(self):
        return True

    def is_stop_state(self):
        return False

    def obstacle_callback(self,data):
        # want to add datapoints with significant width/height
        if data.width >= 75 and data.height >= 75:
            self.obstacles.append(data)
    
    # probably add in distance, too
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