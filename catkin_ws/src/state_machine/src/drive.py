import rospy
import numpy as np
import tf
import tf.transformations as tfm

from me212base.msg import WheelVelCmd, ArduinoData
from apriltags.msg import AprilTagDetections
import me212helper.helper as helper

from state import State
from stop import Stop
import search

class Drive(State):
    def __init__(self, current_input):
        global field_has_far_obstacles

        self.current_input = current_input

        self.count = 0

        self.arrived = False
        #self.far_obstacles = rospy.get_param("field_has_far_obstacles")
        self.far_obstacles = False

        self.tags_in_view = []
        self.detection_poses = {}
        self.tag_centers = {}
        
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        rospy.sleep(0.5)
        
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        self.velcmd_pub = rospy.Publisher("/cmdvel", WheelVelCmd, queue_size = 1)
        self.arduino_data_sub = rospy.Subscriber("/arduino_data", ArduinoData, self.arduino_data_callback, queue_size = 1)
        
        if current_input - int(current_input) == 0.5:
            self.far_obstacles = True
            self.current_input = int(current_input)

        ### Point list
        self.point0 = Pose2D(0.32, 1.0, np.pi/2)
        self.point2a = Pose2D(0.32, 1.22, np.pi/2) # y was 1.52
        self.point2b = Pose2D(0.99, 1.36, np.pi/2)
        self.point3 = Pose2D(1.57, 1.55, 4*np.pi/3)
        self.point4 = Pose2D(0.99, 2.13, np.pi)

        self.target_pose_list = self.get_target_pose_list()
        self.current_target = self.target_pose_list[0]
        self.current_target_index = 0

        self.prev_pos_x = []
        self.prev_pos_y = []
        self.prev_yaw = []

        self.accum_x = []
        self.accum_y = []
        self.accum_theta = []

        self.enc_x = 0
        self.enc_y = 0
        self.enc_theta = 0
        self.hertz = 0
        self.is_safe = 0
        self.wrist_bumper_state = 0

        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

        self.get_current_point()

        self.alpha = 0

        self.x_threshold = 0.1
        self.y_threshold = 0.1
        self.theta_threshold = 0.1

        self.model_tolerance = 0.8

        self.TURN_RIGHT = 1
        self.TURN_LEFT = -self.TURN_RIGHT

    def run(self):
        self.get_current_point()
        print self.current_x, self.current_y, self.current_theta
        if self.current_x == 0:
            print "bad data, try again"
        else:
            target_pose = self.get_target_pose_list()[0]
            self.drive_simple(target_pose)
        rospy.sleep(.01)
        #     self.drive(self.current_target)
        #     if self.current_target.arrived:
        #         if self.current_target_index == len(self.target_pose_list) - 1:
        #             self.arrived = True
        #             return
        #         self.current_target_index += 1
        #         self.current_target = self.target_pose_list[self.current_target_index]
        # else:
        #     self.stop()
    
    def next_input(self):
        return 2 # change later
        #return 8

    def next_state(self):
        if self.current_input == 0: # or self.current_input == 6:
            return search.Search(self.next_input())
        return Stop(self.next_input())

    def is_finished(self):
        return self.arrived

    def is_stop_state(self):
        return False
    
    ## update to include more input options
    def get_target_pose_list(self):
        if self.current_input == 0:
            return [self.point0]
        elif self.current_input == 2:
            if self.far_obstacles:
                return [self.point0, self.point2a]
            #return [self.point0, self.point2b]
            return [self.point0]
        elif self.current_input == 6:
            return [Pose2D(3.05, 1.7, 0)]
        elif self.current_input == 8:
            return [Pose2D(3.05, 1.1, -np.pi/2)]
        return [Pose2D(0, 0, 0)]

    def apriltag_callback(self, data):
        del self.tags_in_view[:]
        corner_x = []
        corner_y = []
        for detection in data.detections:
            self.tags_in_view.append(detection.id)
            self.detection_poses[detection.id] = detection.pose
            for point in detection.corners2d:
                corner_x.append(point.x)
                corner_y.append(point.y)
            center_x = sum(corner_x) / len(corner_x)
            center_y = sum(corner_y) / len(corner_y)
            self.tag_centers[detection.id] = (center_x, center_y)

    def arduino_data_callback(self, data):
        if len(self.accum_x) >= 3:
            self.accum_x.pop(0)
            self.accum_y.pop(0)
            self.accum_theta.pop(0)

        self.accum_x.append(data.deltaX)
        self.accum_y.append(data.deltaY)    
        self.accum_theta.append(data.deltaTheta)
        self.is_safe = data.isSafe      
        self.wrist_bumper_state = data.wristBumperState

        self.enc_x = sum(self.accum_x)/len(self.accum_x)
        self.enc_y = sum(self.accum_y)/len(self.accum_y)
        self.enc_theta = sum(self.accum_theta)/len(self.accum_theta)

    def get_current_point(self):
        apriltag_source_frame = '/apriltag' + str(self.current_input)
        try:
            if self.current_input in self.tags_in_view:
                poselist_tag_cam = helper.pose2poselist(self.detection_poses[self.current_input])
                pose_tag_base = helper.transformPose(pose = poselist_tag_cam,  sourceFrame = '/camera', targetFrame = '/robot_base', lr = self.listener)
                poselist_base_tag = helper.invPoselist(pose_tag_base)
                pose_base_map = helper.transformPose(pose = poselist_base_tag, sourceFrame = apriltag_source_frame, targetFrame = '/map', lr = self.listener)
                helper.pubFrame(self.br, pose = pose_base_map, frame_id = '/robot_base', parent_frame_id = '/map', npub = 1)
                robot_pose3d = helper.lookupTransform(self.listener, '/map', '/robot_base')

                self.current_x = robot_pose3d[0]
                self.current_y = robot_pose3d[1]
                self.current_theta = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
        except:
            print "tag not in view"
            self.current_x = -1
            self.current_y = -1
            self.current_theta = -1 
        print "get_current_point", self.current_x, self.current_y, self.current_theta

    def stop(self):
        wv = WheelVelCmd()

        if not rospy.is_shutdown():
            print '1. Tag not in view, Stop'
            wv.desiredWV_R = -0.1 #-0.05  # right, left
            wv.desiredWV_L = 0.1 #0.05
            wv.desiredWrist = 0.0
            self.velcmd_pub.publish(wv)

        rospy.sleep(.01)

    def drive_simple(self, target_pose2d):
        self.turn_alpha(self.current_input)

        desired_delta_x = self.current_x - target_pose2d.x
        desired_delta_y = self.current_y - target_pose2d.y

        self.drive_simple_x(desired_delta_x)
        self.drive_simple_y(desired_delta_y)
        self.arrived = True

    def drive_simple_x(self, desired_delta_x):
        wv = WheelVelCmd()
        wv.desiredWrist = 0.0

        current_delta_x = 0
        start_enc_count = self.enc_x
        start_x = self.current_x
        desired_x = start_x + desired_delta_x
        k = .1

        if self.alpha >= 0:
            turn_direction = self.TURN_LEFT
        else:
            turn_direction = self.TURN_RIGHT

        num_tries = 0

        while abs(desired_x-self.current_x) > self.x_threshold:
            if num_tries > 0:
                self.turn_alpha(self.current_input)

            while not (abs(desired_delta_x - self.x_threshold) < current_delta_x < abs(desired_delta_x + self.x_threshold)):
                wv.desiredWV_R = k*(desired_delta_x-current_delta_x)
                wv.desiredWV_L = k*(desired_delta_x-current_delta_x)
                self.velcmd_pub.publish(wv)
                current_delta_x += self.enc_x-start_enc_count
                
            wv.desiredWV_R = 0
            wv.desiredWV_L = 0
            self.velcmd_pub.publish(wv)

            self.turn_to_tag(turn_direction)
            self.get_current_point()
            # resolve current point
            est_x = start_x + current_delta_x
            obs_x = self.current_x
            if abs(est_x - obs_x) > self.model_tolerance: # they are too far off
                #self.get_current_point() # take the apriltag data
                self.current_x = est_x

        print "drive_simple_x complete"
        print "took num_tries:", num_tries, "current_x:", self.current_x, "current_alpha", self.current_theta


    def drive_simple_y(self, desired_delta_y):
        wv = WheelVelCmd()
        wv.desiredWrist = 0.0

        current_delta_y = 0
        start_enc_count = self.enc_y
        start_y = self.current_y
        desired_y = start_y + desired_delta_y
        k = .1

        while not (abs(desired_delta_y - self.y_threshold) < current_delta_y < abs(desired_delta_y + self.y_threshold)):
            wv.desiredWV_R = k*(desired_delta_y-current_delta_y)
            wv.desiredWV_L = k*(desired_delta_y-current_delta_y)
            self.velcmd_pub.publish(wv)
            current_delta_y += self.enc_y-start_enc_count
        
        wv.desiredWV_R = 0
        wv.desiredWV_L = 0
        self.velcmd_pub.publish(wv)

        print "drive_simple_y complete"

    def turn_alpha(self, target_tag):
        print self.current_x, self.current_y, self.current_theta
        tag_angles = {}
        tag_angles[6] = 0
        tag_angles[7] = 0
        tag_angles[10] = 0
        tag_angles[2] = np.pi/2
        tag_angles[4] = np.pi/2
        tag_angles[5] = np.pi/2
        tag_angles[0] = np.pi
        tag_angles[1] = np.pi
        tag_angles[3] = np.pi
        tag_angles[8] = 3*np.pi/2
        tag_angles[8] = 3*np.pi/2

        wv = WheelVelCmd()
        wv.desiredWrist = 0.0

        offset = tag_angles[target_tag]
        self.alpha = self.current_theta - offset

        if self.alpha >= 0:
            turn_direction = self.TURN_LEFT
        else:
            turn_direction = self.TURN_RIGHT

        print "alpha", self.alpha, "turn_direction", turn_direction

        current_delta_alpha = 0
        start_enc_theta = self.enc_theta
        start_theta = self.current_theta

        while not (abs(self.alpha - self.theta_threshold) < current_delta_alpha < abs(self.alpha + self.theta_threshold)):
            if turn_direction == self.TURN_LEFT:
                wv.desiredWV_R = 0.05
                wv.desiredWV_L = -0.05
            elif turn_direction == self.TURN_RIGHT:
                wv.desiredWV_R = -0.05
                wv.desiredWV_L = 0.05
            else:
                print "kbai"

            self.velcmd_pub.publish(wv)
            current_delta_alpha += self.enc_theta-start_enc_theta

        wv.desiredWV_R = 0
        wv.desiredWV_L = 0
        self.velcmd_pub.publish(wv)

        print "turn_alpha complete"

    def turn_to_tag(self, direction):
        wv = WheelVelCmd()

        while self.current_input not in self.tags_in_view:
            if direction == self.TURN_RIGHT:
                wv.desiredWV_R = -0.05
                wv.desiredWV_L = 0.05
            elif direction == self.TURN_LEFT:
                wv.desiredWV_R = 0.05
                wv.desiredWV_L = -0.05
            else:
                print "invalid direction"
            wv.desiredWrist = 0.0
            self.velcmd_pub.publish(wv)

        wv.desiredWV_R = 0
        wv.desiredWV_L = 0
        wv.desiredWrist = 0.0
        self.velcmd_pub.publish(wv)

    def turn_to_tag_normal(self, direction):
        wv = WheelVelCmd()
        wv.desiredWrist = 0.0

        while self.current_input not in self.tags_in_view:
            if direction == self.TURN_RIGHT:
                wv.desiredWV_R = -0.05
                wv.desiredWV_L = 0.05
            elif direction == self.TURN_LEFT:
                wv.desiredWV_R = 0.05
                wv.desiredWV_L = -0.05
            else:
                print "invalid direction"
            
            self.velcmd_pub.publish(wv)

        camera_center_x = 320
        camera_x_threshold = 5
        tag_x = self.tag_centers[self.current_input][0]
        while not camera_center_x - camera_x_threshold < tag_x < camera_center_x + camera_x_threshold:
            if direction == self.TURN_RIGHT:
                wv.desiredWV_R = -0.05
                wv.desiredWV_L = 0.05
            elif direction == self.TURN_LEFT:
                wv.desiredWV_R = 0.05
                wv.desiredWV_L = -0.05
            else:
                print "invalid direction"
            self.velcmd_pub.publish(wv)    

        wv.desiredWV_R = 0
        wv.desiredWV_L = 0
        self.velcmd_pub.publish(wv)

    def drive(self, target_pose2d):
        wv = WheelVelCmd()

        print target_pose2d

        if not rospy.is_shutdown():
            
            # 1. get robot pose
            robot_pose3d = helper.lookupTransform(self.listener, '/map', '/robot_base')
            
            # this should never happen
            if robot_pose3d is None:
                print '1. Tag not in view, Stop'
                wv.desiredWV_R = 0.0  # right, left
                wv.desiredWV_L = 0.0
                self.velcmd_pub.publish(wv)  
                return
            
            if len(self.prev_pos_x) < 10:
                self.prev_pos_x.append(robot_pose3d[0])
            else:
                self.prev_pos_x = self.prev_pos_x[1:] + [robot_pose3d[0]]

            if len(self.prev_pos_y) < 10:
                self.prev_pos_y.append(robot_pose3d[1])
            else:
                self.prev_pos_y = self.prev_pos_y[1:] + [robot_pose3d[1]]

            print self.prev_pos_x
            print self.prev_pos_y

            robot_position2d = [self.kalman_filter(self.prev_pos_x), self.kalman_filter(self.prev_pos_y)]
            target_position2d = target_pose2d.pose_list[0:2]
            
            robot_yaw = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
            robot_pose2d = robot_position2d + [robot_yaw]
            
            if len(self.prev_yaw) < 3:
                self.prev_yaw.append(robot_yaw)
            else:
                self.prev_yaw = self.prev_yaw[1:] + [robot_yaw]

            robot_yaw = sum(self.prev_yaw) / len(self.prev_yaw)
            # 2. navigation policy
            # 2.1 if       in the target, 
            # 2.2 else if  close to target position, turn to the target orientation
            # 2.3 else if  in the correct heading, go straight to the target position,
            # 2.4 else     turn in the direction of the target position
            
            pos_delta = np.array(target_position2d) - np.array(robot_position2d)
            robot_heading_vec = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
            heading_err_cross = helper.cross2d( robot_heading_vec, pos_delta / np.linalg.norm(pos_delta) )
            
            pos_delta_unit_vec = pos_delta / np.linalg.norm(pos_delta)

            robot_target_angle = np.arccos(np.clip(np.dot(robot_heading_vec, pos_delta_unit_vec), -1.0, 1.0))
            robot_orientation_angle = helper.diffrad(robot_yaw, target_pose2d.theta)

            print 'robot_position2d', robot_position2d, 'target_position2d', target_position2d
            print 'pos_delta', pos_delta
            print 'robot_yaw', robot_yaw
            print 'norm delta', np.linalg.norm( pos_delta ), 'diffrad', np.fabs(helper.diffrad(robot_yaw, target_pose2d.theta))
            print 'heading_err_cross', heading_err_cross
            print 'robot_target_angle', robot_target_angle
            
            distance_error = 0.08
            angle_error = 0.05

            k_straight =  0.2
            k_turn_target = 0.3
            k_turn_orientation = 0.1

            max_speed = 0.1

            if target_pose2d.arrived or np.linalg.norm(pos_delta) < distance_error and np.fabs(robot_orientation_angle) < angle_error:
                print 'Arrived at target, stopping'
                wv.desiredWV_R = 0  
                wv.desiredWV_L = 0
                target_pose2d.arrived = True
            elif not target_pose2d.aligned and not target_pose2d.arrived_position:
                print 'Turning towards target'
                mult = heading_err_cross / np.fabs(heading_err_cross)
                wv.desiredWV_R = k_turn_target * robot_target_angle * mult
                wv.desiredWV_L = -k_turn_target * robot_target_angle * mult
                if np.fabs(robot_target_angle) < angle_error:
                    target_pose2d.aligned = True
            elif not target_pose2d.arrived_position and np.linalg.norm(pos_delta) > distance_error:
                print 'Driving towards target'
                wv.desiredWV_R = min(k_straight * np.linalg.norm(pos_delta), max_speed)
                wv.desiredWV_L = min(k_straight * np.linalg.norm(pos_delta), max_speed)
            else:
                print 'Turning towards orientation'
                mult = robot_orientation_angle / np.fabs(robot_orientation_angle)
                wv.desiredWV_R = -k_turn_orientation * robot_orientation_angle * mult
                wv.desiredWV_L = k_turn_orientation * robot_orientation_angle * mult
                target_pose2d.arrived_position = True

            print
            # TODO: replace with real controller

            '''
            k = np.linalg.norm(pos_delta)
            print k
            # print "k", k
            # print "other", np.fabs(helper.diffrad(robot_yaw, target_pose2d.theta))
            # general strategy:
            # turn towards target point
            # drive straight to target point
            # turn towards final orientation
            if target_pose2d.arrived or k < 0.4 and np.fabs(helper.diffrad(robot_yaw, target_pose2d.theta)) < 0.05:
                print 'Case 2.1 Stop'
                wv.desiredWV_R = 0  
                wv.desiredWV_L = 0
                target_pose2d.arrived = True
            elif not target_pose2d.arrived_position or np.fabs( heading_err_cross ) > 0.3:
                print 'Case 2.2 Straight forward'
                wv.desiredWV_R = 0.1 * k
                wv.desiredWV_L = 0.1 * k
            else:
                print 'Case 2.1 Turning'
                if k < 0.4:
                    target_pose2d.arrived_position = True
                print k
                mult = heading_err_cross / np.fabs(heading_err_cross)
                wv.desiredWV_R = 0.1 * k * mult * 0.5
                wv.desiredWV_L = -0.1 * k * mult * 0.5
            ''' 

            wv.desiredWrist = 0.0
            self.velcmd_pub.publish(wv)
            
            self.count += 1
            rospy.sleep(0.02)

    def kalman_filter(self, z):
        # intial parameters
        n_iter = len(z)
        sz = (n_iter,)

        Q = 1e-5 # process variance

        # allocate space for arrays
        xhat=np.zeros(sz)      # a posteri estimate of x
        P=np.zeros(sz)         # a posteri error estimate
        xhatminus=np.zeros(sz) # a priori estimate of x
        Pminus=np.zeros(sz)    # a priori error estimate
        K=np.zeros(sz)         # gain or blending factor

        R = 0.15**2 # estimate of measurement variance, change to see effect

        # intial guesses
        xhat[0] = z[0]
        P[0] = z[0] + 0.1 # estimate we're 10 cm away from our initial guess

        for k in range(1,n_iter):
            # time update
            xhatminus[k] = xhat[k-1]
            Pminus[k] = P[k-1]+Q

            # measurement update
            K[k] = Pminus[k]/( Pminus[k]+R )
            xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k])
            P[k] = K[k]*Pminus[k]

        return Pminus[-1]

    def __str__(self):
        return "Drive(%s)" % (self.current_input)

class Pose2D():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

        self.pose_list = [x, y, theta]

        self.arrived = False
        self.aligned = False
        self.arrived_position = False

    def __str__(self):
        return "Pose2d: %s %s %s" % (self.x, self.y, self.theta)