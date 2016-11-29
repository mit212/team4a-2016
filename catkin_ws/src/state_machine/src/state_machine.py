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
        self.thread = threading.Thread(target = self.run_state_machine)
            
        self.thread.start()
        
        rospy.sleep(1)
        
    def run_state_machine(self):
        current_state = Start(0)
        while not current_state.is_stop_state():
            print "current state", current_state
            while not current_state.is_finished():
                current_state.run()
            print "next input", current_state.next_input()
            current_state = current_state.next_state()
        print "done"
        
    
def main():
    rospy.init_node('state_machine', anonymous=True)
    state_machine = StateMachine()
    rospy.spin()

if __name__=='__main__':
    main()
