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
from drive import Drive
import search

class DetectObstacles(State):
    def __init__(self, current_input):

        self.current_input = current_input
        
        self.obstacles = []
        
        self.object_sub = rospy.Subscriber("/object_info", DetectedObject, self.obstacle_callback, queue_size = 1)
        rospy.sleep(3)
        field_has_far_obstacles = self.determine_obstacles()

        print "far obstacles:", field_has_far_obstacles

        rospy.set_param("field_has_far_obstacles", field_has_far_obstacles)

        self.classified_obstacles = False

    def run(self):
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
        if data.width >= 120 and data.height >= 120:
            self.obstacles.append(data)
    
    # probably add in distance, too
    def determine_obstacles(self):
        relevant_obstacles = self.obstacles[-5:]
        print relevant_obstacles
        far_obs_count = 0
        near_obs_count = 0
        for obstacle in relevant_obstacles:
            if obstacle.center_x >= 165 and obstacle.center_x <= 260 and obstacle.center_y >= 320 and obstacle.center_y <= 390:
                if obstacle.width >= 350 and obstacle.height >= 170:
                    return True
        return False

    def __str__(self):
        return "DetectObstacles(%s)" % (self.current_input)