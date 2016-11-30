#!/usr/bin/python

# 2.12 Lab 2 me212bot: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016

import rospy
import tf
import numpy as np
import threading
import serial
import pdb
import traceback
import sys

from visualization_msgs.msg import Marker
from me212base.msg import WheelVelCmd, ArduinoData
from geometry_msgs.msg import Point, Pose, Twist
import me212helper.helper as helper

port = '/dev/ttyACM0'

class Arduino():
    def __init__(self, port = '/dev/ttyACM0'):
        self.comm = serial.Serial(port, 115200, timeout = 5)
        self.sendbuff = []
        self.readbuff = ''
        
        self.thread = threading.Thread(target = self.loop)
        self.thread.start()
        
        self.prevtime = rospy.Time.now()
        
        self.velcmd_sub = rospy.Subscriber("/cmdvel", WheelVelCmd, self.cmdvel)
        self.arduino_data_pub = rospy.Publisher("/arduino_data", ArduinoData, queue_size = 1)

    def cmdvel(self, msg):  
        self.comm.write("%f,%f,%f,\n" % (msg.desiredWV_R, msg.desiredWV_L, msg.desiredWrist))
    
    # loop() is for reading odometry from Arduino and publish to rostopic.
    def loop(self):
        while not rospy.is_shutdown():
            # 1. get a line of string that represent current odometry from serial
            serialData = self.comm.readline()
            #print serialData
            
            # 2. parse the string e.g. "0.1,0.2,0.1" to doubles
            splitData = serialData.split(',')
            #print splitData

            try:
                x     = float(splitData[0])
                y     = float(splitData[1])
                theta = float(splitData[2])
                isSafe = int(splitData[3])
                wristBumperState = int(splitData[4])
                hz    = 1.0 / (rospy.Time.now().to_sec() - self.prevtime.to_sec())

                ad = ArduinoData()
                ad.deltaX = x
                ad.deltaY = y
                ad.deltaTheta = theta #self.constrain_theta(theta)
                ad.hertz = hz
                ad.isSafe = isSafe
                ad.wristBumperState = wristBumperState
                self.arduino_data_pub.publish(ad)

                print 'x=', x, ' y=', y, ' theta =', self.constrain_theta(theta), ' hz =', hz, ' isSafe =', isSafe, ' wristBumperState =', wristBumperState; 
                    
                self.prevtime = rospy.Time.now()
                
            except:
                # print out msg if there is an error parsing a serial msg
                print 'Cannot parse', splitData
                ex_type, ex, tb = sys.exc_info()
                traceback.print_tb(tb)

    def constrain_theta(self, theta_in):
        theta_out = theta_in

        while theta_out > 2*np.pi:
            theta_out -= 2*np.pi
        while theta_out < 0:
            theta_out += 2*np.pi

        return theta_out 

def main():
    rospy.init_node('me212base_node', anonymous=True)
    arduino = Arduino()
    rospy.spin()
    
if __name__=='__main__':
    main()
    
