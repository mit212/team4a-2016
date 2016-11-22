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

        self.zero_pos = 0
        self.position = 0

        self.thread = threading.Thread(target = self.loop)
        self.thread.start()
        rospy.sleep(1)
        
    def loop(self):
        rospy.sleep(0.2)
        self.zero_pos = self.position
        self.current_pos = 0

        run = True
        count = 0
        while not rospy.is_shutdown():
            #print count
            if count >= 3:
                run = False
            
            if run:
                # self.exec_joint1_pub.publish(std_msgs.msg.Float64(0.0))
                # print "position:", self.position, "velocity:", self.velocity
                # rospy.sleep(2)
                # self.exec_joint1_pub.publish(std_msgs.msg.Float64(1.0))
                # print "position:", self.position, "velocity:", self.velocity
                # rospy.sleep(2)
                # self.exec_joint1_pub.publish(std_msgs.msg.Float64(2.0))
                # print "position:", self.position, "velocity:", self.velocity
                # rospy.sleep(2)
                # while abs(self.current_pos) < 3:
                #     self.exec_joint1_pub.publish(std_msgs.msg.Float64(3.0))
                #     print "position:", self.position, "abs pos:", self.current_pos
                # print "checkpoint"
                # self.stop()
                # while True:
                #     pass
                # while self.current_pos > -3:
                #     self.exec_joint1_pub.publish(std_msgs.msg.Float64(-3.0))
                #     print "position:", self.position, "abs pos:", self.current_pos
                # rospy.sleep(2)

                self.run_distance(10, 10.0)
                self.run_distance(10, -2.0)
                self.stop()
                run = False

            else:
                self.exec_joint1_pub.publish(std_msgs.msg.Float64(0))
            
            count +=1

        rospy.sleep(0.3)
    
    def end_effector_callback(self, data):
        self.velocity = data.velocity
        self.position = data.current_pos
        self.load = data.load
        self.current_pos = self.position-self.zero_pos
        #print "something", self.velocity
    def stop(self):
        self.exec_joint1_pub.publish(std_msgs.msg.Float64(0))

    def run_distance(self, distance, speed):
        last_pos = self.position
        elapsed_distance = 0
        self.exec_joint1_pub.publish(std_msgs.msg.Float64(speed))
        
        while elapsed_distance < distance:
            increment = abs(self.position-last_pos)
            print "                         elapsed distance:", elapsed_distance
            print "check increment:", increment
            if increment < 3.0:
                elapsed_distance += increment
            last_pos = self.position
            rospy.sleep(0.01)
        
        self.stop()
        
        print "run_distance", distance, "complete" 

def main():
    rospy.init_node("run_planning")
    run_planning = RunPlanning()
    rospy.spin()

if __name__=="__main__":
    main()

    




