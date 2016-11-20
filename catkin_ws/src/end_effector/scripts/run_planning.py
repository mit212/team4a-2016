#!/usr/bin/python

# 2.12 Lab 5 trajectory planning
# Peter Yu Oct 2016
import rospy
import planner
import threading
import std_msgs.msg, sensor_msgs.msg, dynamixel_msgs.msg
#import numpy as np

class RunPlanning():
    def __init__(self):
        self.exec_joint1_pub = rospy.Publisher('/joint1_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.robotjoints = rospy.Subscriber('/joint1_controller/state', dynamixel_msgs.msg.JointState, self.end_effector_callback, queue_size=1)
        print "this is robot joints:", self.robotjoints 

        self.position = 0
        self.velocity = 0

        self.thread = threading.Thread(target = self.loop)
        self.thread.start()
        rospy.sleep(1)
        
    def loop(self):
        #if use_real_arm:
        run = True
        count = 0
        while not rospy.is_shutdown():
            print count
            if count >= 3:
                run = False
            
            if run:
                self.exec_joint1_pub.publish(std_msgs.msg.Float64(0.0))
                print "position:", self.position, "velocity:", self.velocity
                rospy.sleep(2)
                self.exec_joint1_pub.publish(std_msgs.msg.Float64(1.0))
                print "position:", self.position, "velocity:", self.velocity
                rospy.sleep(2)
                self.exec_joint1_pub.publish(std_msgs.msg.Float64(2.0))
                print "position:", self.position, "velocity:", self.velocity
                rospy.sleep(2)
                self.exec_joint1_pub.publish(std_msgs.msg.Float64(3.0))
                print "position:", self.position, "velocity:", self.velocity
                rospy.sleep(2)
                self.exec_joint1_pub.publish(std_msgs.msg.Float64(-3.0))
                print "position:", self.position, "velocity:", self.velocity
                rospy.sleep(2)
            else:
                self.exec_joint1_pub.publish(std_msgs.msg.Float64(0))
            
            count +=1

        rospy.sleep(0.3)
    
    def end_effector_callback(self, data):
        self.velocity = data.velocity
        self.position = data.current_pos
        #print "something", self.velocity

def main():
    rospy.init_node("run_planning")
    run_planning = RunPlanning()
    rospy.spin()

if __name__=="__main__":
    main()

    




