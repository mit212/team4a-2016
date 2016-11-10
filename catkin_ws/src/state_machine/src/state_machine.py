#!/usr/bin/python

import rospy
import tf
import numpy as np
import threading
import serial
import pdb
import traceback
import sys
import tf.transformations as tfm

import state
from start import Start
import search

class StateMachine():
    def __init__(self):
        #self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)   ##
        pass
    
def main():
    rospy.init_node('state_machine', anonymous=True)
    rospy.spin()
    
    currentState = Start(0)
    while not currentState.isStopState():
        while not currentState.isFinished():
            currentState.run()
        print currentState.nextInput()
        currentState = currentState.nextState()(currentState.nextInput())
        # publish things here
    

if __name__=='__main__':
    main()
