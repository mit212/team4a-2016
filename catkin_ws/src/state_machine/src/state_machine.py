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
        
        self.thread = threading.Thread(target = self.run_state_machine)
            
        self.thread.start()
        
        rospy.sleep(1)
        
    def run_state_machine(self):
        print "running"
        currentState = Start(0)
        while not currentState.isStopState():
            print currentState.currentInput
            while not currentState.isFinished():
                currentState.run()
            print currentState.nextInput()
            currentState = currentState.nextState()(currentState.nextInput())
            # publish things here
        
    
def main():
    rospy.init_node('state_machine', anonymous=True)
    state_machine = StateMachine()
    rospy.spin()

if __name__=='__main__':
    main()
