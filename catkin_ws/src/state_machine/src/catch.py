import rospy
import tf
import threading
import serial
import pdb
import traceback
import sys
import tf.transformations as tfm

from end_effector.msg import end_effector
import me212helper.helper as helper
import std_msgs.msg, sensor_msgs.msg, dynamixel_msgs.msg
from me212base.msg import WheelVelCmd, ArduinoData

from state import State
from stop import Stop
#from drive import Drive

#catch state manages three motions: extending the linear actuator, 
#                                   rotating the wrist 90 degrees, 
#                                   and retracting the linear actuator


class Catch(State):
    def __init__(self, current_input):
        self.zero_pos = 0
        self.position = 0
        self.current_pos = 0
        self.load = 0
        self.catch_success = False
        self.current_input = current_input

        self.WRIST_UP = 0
        self.WRIST_DOWN = 1

        self.wrist_bumper_state = 0
        self.PRESSED = 1

        self.exec_joint1_pub = rospy.Publisher('/joint1_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.robotjoints = rospy.Subscriber('/joint1_controller/state', dynamixel_msgs.msg.JointState, self.end_effector_callback, queue_size=1)
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)
        self.arduino_data_sub = rospy.Subscriber("/arduino_data", ArduinoData, self.arduino_data_callback, queue_size = 1)
        rospy.sleep(1)

        self.thread = threading.Thread(target = self.run)
        self.thread.start()
        rospy.sleep(1)
        
        
    def run(self):
        rospy.sleep(0.2)
        self.zero_arm()
        self.zero_pos = self.position
        run = True

        wv = WheelVelCmd()

        while (not rospy.is_shutdown()) and (not self.catch_success):
            if not self.catch_success:
                self.move_wrist(self.WRIST_UP)   
                self.run_distance(14, 20.0)
                self.stop()

                self.move_wrist(self.WRIST_DOWN)
                self.run_distance(14, -20.0)
                rospy.sleep(.5)
                self.move_wrist(self.WRIST_UP)
                self.stop()

                self.catch_success = True
        rospy.sleep(0.3)
    
    def next_input(self):
        return self.current_input # CHANGE THIS

    def next_state(self):
        #return Drive(self.next_input())
        return Stop(self.next_input())

    def is_finished(self):
        return self.catch_success

    def is_stop_state(self):
        return False

    def end_effector_callback(self, data):
        self.velocity = data.velocity
        self.position = data.current_pos
        self.load = data.load
        #self.current_pos = self.position-self.zero_pos

    def arduino_data_callback(self, data):  
        self.wrist_bumper_state = data.wristBumperState


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
            #print "elapsed distance:", elapsed_distance
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

    def zero_arm(self):
        while not self.wrist_bumper_state == self.PRESSED:
            self.exec_joint1_pub.publish(std_msgs.msg.Float64(-4))
            rospy.sleep(0.01)
        self.exec_joint1_pub.publish(std_msgs.msg.Float64(0))
        rospy.sleep(0.01)

    def __str__(self):
        return "Catch(%s)" % (self.current_input)

def main():
    rospy.init_node("run_planning")
    run_planning = RunPlanning()
    rospy.spin()

if __name__=="__main__":
    main()
