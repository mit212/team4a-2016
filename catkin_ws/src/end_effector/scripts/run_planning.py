#!/usr/bin/python

# 2.12 Catch Testing
# Jesse Chang

import rospy
import planner
import threading
import std_msgs.msg, sensor_msgs.msg, dynamixel_msgs.msg
from me212base.msg import WheelVelCmd
#import numpy as np

class RunPlanning():
    def __init__(self):
        self.zero_pos = 0
        self.position = 0
        self.current_pos = 0
        self.load = 0

        self.exec_joint1_pub = rospy.Publisher('/joint1_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.robotjoints = rospy.Subscriber('/joint1_controller/state', dynamixel_msgs.msg.JointState, self.end_effector_callback, queue_size=1)
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)
        rospy.sleep(1)

        self.thread = threading.Thread(target = self.loop)
        self.thread.start()
        rospy.sleep(1)
        
    def loop(self):
        rospy.sleep(0.2)
        self.zero_pos = self.position


        run = True
        count = 0

        wv = WheelVelCmd()
        WRIST_UP = 0
        WRIST_DOWN = 1

        while (not rospy.is_shutdown()) and run:
            #print count
            if count >= 3:
                run = False
            
            if run:
                self.move_wrist(WRIST_UP)   
                self.run_distance(15, 20.0)
                self.stop()
                self.move_wrist(WRIST_DOWN)
                rospy.sleep(.5)
                self.run_distance(15, -20.0)
                rospy.sleep(.5)
                self.move_wrist(WRIST_UP)
                self.stop()

                run = False

            
            count +=1

        rospy.sleep(0.3)
    
    def end_effector_callback(self, data):
        self.velocity = data.velocity
        self.position = data.current_pos
        self.load = data.load
        #self.current_pos = self.position-self.zero_pos
        #print "something", self.velocity
    def stop(self):
        self.exec_joint1_pub.publish(std_msgs.msg.Float64(0))

    def run_distance(self, distance, speed):
        #returns 0 if success, 1 if general error, >1 errorID

        #max distance is 15 to traverse the length of the rack gear
        #make sure that 0 <= distance <= 15

        #speed is a float, positive for CCW, negative for CW, 20 is max...

        if not (distance >=0 and distance <= 15):
            print "distance argument invalid"
            while 1:
                pass
            return 1

        last_pos = self.position
        last_load = self.load
        load_inc = last_load
        elapsed_distance = 0
        self.exec_joint1_pub.publish(std_msgs.msg.Float64(speed))
        
        while elapsed_distance < distance:
            self.exec_joint1_pub.publish(std_msgs.msg.Float64(speed))
            increment = abs(self.position-last_pos)
            load_inc = abs(self.load - last_load)
            print "                         elapsed distance:", elapsed_distance
            #print "check increment:", increment
            # print "check load:", self.load, "load inc:", load_inc
            # if load_inc > 0.3:
            #     print "bad"
            #     self.stop()
            #     while 1:
            #         pass

            if increment < 3.0:
                elapsed_distance += increment
            last_pos = self.position
            last_load = self.load
            rospy.sleep(0.01)
        
        self.stop()
        
        print "run_distance", distance, "complete" 
        return 0

    def move_wrist(self, position):
        wv = WheelVelCmd()
        wv.desiredWV_R = 0  # don't move...
        wv.desiredWV_L = 0
        wv.desiredWrist = position
        self.velcmd_pub.publish(wv)  

        return 0

def main():
    rospy.init_node("run_planning")
    run_planning = RunPlanning()
    rospy.spin()

if __name__=="__main__":
    main()

    




