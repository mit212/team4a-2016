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
from std_msgs.msg import Bool
from me212cv.msg import DetectedObject
import me212helper.helper as helper

from state import State
from stop import Stop
from drive import Drive
import search

class PrepareToCatchBox(State):
    def __init__(self, current_input):

        self.current_input = current_input
        
        self.pokemon = []
        
        self.object_sub = rospy.Subscriber("/object_info", DetectedObject, self.pokemon_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)

        self.pikachu = True

        self.found_pokemon = False
        self.should_stop = False

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
        return Stop(self.next_input())

    def is_finished(self):
        return self.should_stop

    def is_stop_state(self):
        return False

    def pokemon_callback(self,data):
        # want to add datapoints with significant width/height
        if self.pikachu:
            if data.width >= 40 and data.width <= 70 and data.height >= 40 and data.height <= 70:
                if data.center_x > 300 and data.center_x < 400 and data.center_y > 350 and data.center_y < 450:
                    self.found_pokemon = True
        else:
            if data.width >= 60 and data.height >= 75:
                if data.center_y > 380 and data.center_y < 400:
                    self.found_pokemon = True
    
    # probably add in distance, too
    def determine_pokemon(self):
        relevant_pokemon = self.pokemon[-5:]
        for obstacle in relevant_obstacles:
            if obstacle.center_x >= 165 and obstacle.center_x <= 200 and obstacle.center_y >= 360 and obstacle.center_y <= 390:
                if obstacle.width >= 350 and obstacle.height >= 170:
                    return True
        return False

    def __str__(self):
        return "DetectObstacles(%s)" % (self.current_input)