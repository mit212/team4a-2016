#!/usrbin/env python
#import end_effector.msg
import set_end_effector.srv
import rospy

rospy.init_node("end_effector_state_publisher")
joint1_sub = message_filters.Subscriber("/joint1_controller/state", dynamixel_msgs.msg.JointState)

linear_pos_sub = message_filters.Subscriber("/end_effector_controller/state", end_effector.msg.JointState)
rotational_pos_sub
joint_pub = rospy.Publisher("/joint_states", sensor_msgs.msg.JointState,  queue_size=1) 


def handle_end_effector_claw(req):
	print "returning"
