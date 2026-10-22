#!/usr/bin/env python3

"""
Start ROS node to publish linear and angular velocities to mymobibot 
in order to perform Kalman Filtering using IMU and sonar measurements.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t
import matplotlib.pyplot as plt

# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class mymobibot_localization():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):
        # to plot diagrams
        self.x_positions=[]
        self.y_positions=[]
        self.theta_angle=[]
        self.x_sensors=[]
        self.y_sensors=[]

        # linear and angular velocity
        self.velocity = Twist()
        # joints' states
        self.joint_states = JointState()
        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()
        rospy.on_shutdown(self.turn_off)

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('/mymobibot/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        # Publishers to plot
        self.x_pos = rospy.Publisher("/x_pos", Float64, queue_size=100)
        self.y_pos = rospy.Publisher("/y_pos", Float64, queue_size=100)
        self.theta_ang = rospy.Publisher("/theta_ang", Float64, queue_size=100)
        self.x_sens = rospy.Publisher("/x_sens", Float64, queue_size=100)
        self.y_sens = rospy.Publisher("/y_sens", Float64, queue_size=100)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])

    def imu_callback(self, msg):
        # ROS callback to get the /imu

        self.imu = msg
        # (e.g. the orientation of the robot wrt the global frome is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frome is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frome is stored in :: self.imu.linear_acceleration)

        #quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #(roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)
        (roll, pitch, self.imu_yaw) = quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def sonar_front_callback(self, msg):
        # ROS callback to get the /sensor/sonar_F

        self.sonar_F = msg
        # (e.g. the distance from sonar_front to an obstacle is stored in :: self.sonar_F.range)

    def sonar_frontleft_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FL

        self.sonar_FL = msg
        # (e.g. the distance from sonar_frontleft to an obstacle is stored in :: self.sonar_FL.range)

    def sonar_frontright_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FR

        self.sonar_FR = msg
        # (e.g. the distance from sonar_frontright to an obstacle is stored in :: self.sonar_FR.range)

    def sonar_left_callback(self, msg):
        # ROS callback to get the /sensor/sonar_L

        self.sonar_L = msg
        # (e.g. the distance from sonar_left to an obstacle is stored in :: self.sonar_L.range)

    def sonar_right_callback(self, msg):
        # ROS callback to get the /sensor/sonar_R

        self.sonar_R = msg
        # (e.g. the distance from sonar_right to an obstacle is stored in :: self.sonar_R.range)

    def publish(self):

        # set configuration
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        print("The system is ready to execute the Kalman Filter algorithm...")
        
        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        # initialize
        x_t = 0
        y_t = 0
        theta_t = 0
        vx_t = 0
        s_w = 0.002
        s_ax = 0.002
        time_b = time_now
        L1 = 0.05
        L2 = 0.018
        L3 = 0.1
        L4 = 0.2

        xhat = 0
        yhat = 0
        vel_x = 0
        P = np.zeros((3,3))
        # Access the robot's yaw angle (imu_yaw)
        angle = self.imu_yaw
        print("Current yaw angle:", angle)
        thetahat = angle + np.pi/2
        while not rospy.is_shutdown():
            angle = self.imu_yaw
            print("Current yaw angle:", angle)
            # Calculate time interval (in case is needed)
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9

            #
            # Predict
            #
            angle_vel = self.imu.angular_velocity.z
            ax = self.imu.linear_acceleration.x

            # if angle >= 0:
            #     theta = angle + np.pi/2
            # elif angle < 0 and angle >= -np.pi/2:
            #     theta = np.pi/2 + angle
            # elif angle < -np.pi/2 and angle > -np.pi:
            #     theta = angle + np.pi/2
            theta = angle + np.pi/2

            sigma_x = 1/2*s_ax*np.cos(theta)*(dt**2)
            sigma_y = 1/2*s_ax*np.sin(theta)*(dt**2)
            sigma_theta = s_w * dt

            Cw = np.array( [[sigma_x**2,0,0],\
                            [0,sigma_y**2,0],\
                            [0,0,sigma_theta**2]])

            xhat = xhat + (vel_x*dt+(1/2)*ax*dt**2)*np.cos(theta)
            yhat = yhat + (vel_x*dt+(1/2)*ax*dt**2)*np.sin(theta)
            thetahat = thetahat + angle_vel*dt
            vel_x += ax*dt

            print("Difference x: ",(vel_x*dt+(1/2)*ax*dt**2)*np.cos(theta),"Difference y: ", (vel_x*dt+(1/2)*ax*dt**2)*np.sin(theta))
            print("xhat: ",xhat, " --- yhat: ", yhat)


            #linearization_of_state_space
            A = np.array( [ [1, 0, -(vel_x*dt+1/2*ax*dt**2)*np.sin(theta)],\
                            [0, 1, (vel_x*dt+1/2*ax*dt**2)*np.cos(theta)],\
                            [0, 0, 1]])

            #prediction probability
            P = np.dot(np.dot(A,P), np.transpose(A)) + Cw


            #
            # Update
            #
            sonar_F = self.sonar_F.range 
            sonar_FL = self.sonar_FL.range 
            sonar_FR = self.sonar_FR.range 
            sonar_R = self.sonar_R.range 
            sonar_L = self.sonar_L.range 
            # angle = self.imu.orientation.z
            dist = 0 # the distance from wall 
            same_wall = False
            poss_x = 0 
            poss_y = 0
            phi = 0.46365
            theta_0 = np.pi/4

            
            # dist_FR = (2/np.sqrt(2))*dist_R + 0.018
            # print(np.arcsin(angle), " --- ", angle)

            #
            # For the front and the the front left sonars
            #
            if sonar_F < 2 and sonar_FL < 2:
                # we find the angle between the sonar_F and the wall using the robot's orientation and in the F, FR, wall triangle
                # if it is larger than np.pi/2 we keep only angle - np.pi/2
                if angle > 0 and angle < np.pi/2:
                    wallangle = angle
                elif angle > np.pi/2:
                    wallangle = angle - np.pi/2
                elif angle < 0 and angle > -np.pi/2:
                    wallangle = np.pi/2 + angle
                else:
                    wallangle = angle + np.pi

                l_FL = sonar_FL + 0.14142
                l_F = sonar_F + 0.1
                l_wall = sqrt(l_FL**2+l_F**2 - 2*l_FL*l_F*cos(np.pi/4))

                # a is the wall angle that we find  with the sonars in the triangle with edges sonar_F, sonar_FL and the wall edge
                a = np.arccos((l_F**2+l_wall**2-l_FL**2)/(2*l_F*l_wall))

                print(a, " --- ", wallangle)
                dist = np.sin(wallangle)*(sonar_F + L4)
                print(2-dist)
                # if the two angles are almost the same then the two sonars are detecting the same wall
                if  (wallangle-0.25<a and a < wallangle+0.25) or (wallangle-0.25<a-np.pi/2 and a-np.pi/2 < wallangle+0.25):
                    same_wall = True
                    print("same for FL and F")

                    if angle <= theta_0 and angle >= 0:
                        print("1")
                        if a >= np.pi/2-0.07:
                            print("top wall 1")
                            poss_y = 2 - dist
                        else: 
                            print("left wall 1")
                            poss_x = -(2 - dist)
                    elif angle <= np.pi/2 and angle > theta_0 and angle >= 0:
                        print("left wall 2")
                        poss_x = -(2 - dist)
                    elif angle <= np.pi/2 + theta_0 and angle > np.pi/2 and angle >= 0:
                        if a >= np.pi/2 - 0.07:
                            print("left wall 3")
                            poss_x = -(2 - dist)
                        else: 
                            print("bottom wall 1")
                            poss_y = -(2 - dist)
                    elif angle <= np.pi and angle > np.pi/2 + theta_0 and angle >= 0:
                        print("bottom wall 2")
                        poss_y = -(2 - dist)
                    elif angle >= -np.pi/2 and angle < 0:
                        if a >= np.pi/2:
                            print("right wall 1")
                            poss_x = 2 - dist
                        else: 
                            print("top wall 2")
                            poss_y = 2 - dist
                    elif angle >= -np.pi/2 - theta_0 and angle < -np.pi/2 and angle < 0:
                        print("right wall 2")
                        poss_x = 2 - dist
                    elif angle >= -np.pi and angle < -np.pi/2 - theta_0 and angle < 0:
                        if a >= np.pi/2:
                            print("bottom wall 3")
                            poss_y = -(2 - dist)
                        else: 
                            print("right wall 3") 
                            poss_x = 2 - dist
                else:
                    dist = np.sin(np.pi/2 - wallangle)*(sonar_F + L4)
                    print(2-dist)
                    # the two sonars detect different walls
                    if angle <= np.pi/2 and angle >= 0:
                        print("F: top wall, FL: left wall")
                        poss_y = 2 - dist
                    elif angle <= np.pi and angle > np.pi/2:
                        print("F: left wall, FL: bottom wall")
                        poss_x = -(2 - dist)
                    elif angle < 0 and angle >= -np.pi/2:
                        print("F: right wall, FL: top wall")
                        poss_x = 2 - dist
                    elif angle < -np.pi/2 and angle > -np.pi:
                        print("F: bottom wall, FL: bottom wall")
                        poss_y = -(2 - dist)

            #
            # For the front and the front right sonars
            #
            if sonar_F < 2 and sonar_FR < 2:
                # we find the angle between the sonar_F and the wall using the robot's orientation and in the F, FR, wall triangle
                # if it is larger than np.pi/2 we keep only angle - np.pi/2
                if angle > 0 and angle < np.pi/2:
                    wallangle = np.pi/2 - angle
                elif angle > np.pi/2:
                    wallangle = np.pi - angle
                elif angle < 0 and angle > -np.pi/2:
                    wallangle = -angle
                else:
                    wallangle = -angle - np.pi/2

                l_FR = sonar_FR + 0.14142
                l_F = sonar_F + 0.1
                l_wall = sqrt(l_FR**2+l_F**2 - 2*l_FR*l_F*cos(np.pi/4))

                # a is the wall angle that we find  with the sonars in the triangle with edges sonar_F, sonar_FL and the wall edge
                a = np.arccos((l_F**2+l_wall**2-l_FR**2)/(2*l_F*l_wall))

                print(a, " --- ", wallangle)
                dist = np.sin(wallangle)*(sonar_F + L4)

                # if the two angles are almost the same then the two sonars are detecting the same wall
                if  (wallangle-0.25<a and a < wallangle+0.25) or (wallangle-0.25<a-np.pi/2 and a-np.pi/2 < wallangle+0.25):
                    same_wall = True
                    print("same for FR and F")

                    if angle <= np.pi/2 and angle >= 0:
                        if a >= np.pi/2 and poss_x == 0:
                            print("left wall 1")
                            poss_x = -(2 - dist)
                        elif poss_y == 0: 
                            print("top wall 2")
                            poss_y = 2 - dist
                    elif angle <= np.pi/2 + theta_0 and angle > np.pi/2 and poss_x == 0:
                        print("left wall 2")
                        poss_x = -(2 - dist)
                    elif angle <= np.pi and angle > np.pi/2 + theta_0:
                        if a >= np.pi/2 and poss_y == 0:
                            print("bottom wall 1")
                            poss_y = -(2 - dist)
                        elif poss_x == 0: 
                            print("left wall 3") 
                            poss_x = -(2 - dist)
                    elif angle >= -theta_0 and angle < 0:
                        if a >= np.pi/2-0.07 and poss_y == 0:
                            print("top wall 2")
                            poss_y = 2 - dist
                        elif poss_x == 0: 
                            print("right wall 1")
                            poss_x = 2 - dist
                    elif angle >= -np.pi/2 and angle < -theta_0 and poss_x == 0:
                        print("right wall 2")
                        poss_x = 2 - dist
                    elif angle >= -np.pi/2 - theta_0 and angle < -np.pi/2:
                        if a >= np.pi/2 - 0.07 and poss_x == 0:
                            print("right wall 3")
                            poss_x = 2 - dist
                        elif poss_y == 0: 
                            print("bottom wall 2")
                            poss_y = -(2 - dist)
                    elif angle > -np.pi and angle < -np.pi/2 - theta_0 and poss_y == 0:
                        print("bottom wall 3")
                        poss_y = -(2 - dist)
                else:
                    # the two sonars detect different walls
                    if angle <= np.pi/2 and angle >= 0 and poss_x == 0:
                        print("F: left wall, FR: top wall")
                        poss_x = -(2 - dist)
                    elif angle <= np.pi and angle > np.pi/2 and poss_y == 0:
                        print("F: bottom wall, FR: left wall")
                        poss_y = -(2 - dist)
                    elif angle < 0 and angle >= -np.pi/2 and poss_y == 0:
                        print("F: top wall, FR: right wall")
                        poss_y = 2 - dist
                    elif angle < -np.pi/2 and angle > -np.pi and poss_x == 0:
                        print("F: right wall, FR: bottom wall")
                        poss_x = 2 - dist

            #
            # For the left and the the front left sonars
            #
            if sonar_L < 2 and sonar_FL < 2:
                # we find the angle between the sonar_L and the wall using the robot's orientation and in the L, FL, wall triangle
                # if it is larger than np.pi/2 we keep only angle - np.pi/2
                if angle > 0 and angle < np.pi/2:
                    wallangle = np.pi/2 - angle
                elif angle > np.pi/2:
                    wallangle = np.pi - angle
                elif angle < 0 and angle > -np.pi/2:
                    wallangle = -angle
                else:
                    wallangle = -angle - np.pi/2

                l_FL = sonar_FL + 0.14142
                l_L = sonar_L + 0.1
                l_wall = sqrt(l_FL**2+l_L**2 - 2*l_FL*l_L*cos(np.pi/4))

                # a is the wall angle that we find  with the sonars in the triangle with edges sonar_L, sonar_FL and the wall edge
                a = np.arccos((l_L**2+l_wall**2-l_FL**2)/(2*l_L*l_wall))

                print(a, " --- ", wallangle)

                # distance from robot's center to the wall detected by left sonar
                dist = np.sin(wallangle)*(sonar_L + L3)

                # if the two angles are almost the same then the two sonars are detecting the same wall
                if  (wallangle-0.25<a and a < wallangle+0.25) or (wallangle-0.25<a-np.pi/2 and a-np.pi/2 < wallangle+0.25):
                    same_wall = True
                    print("same for FL and L")

                    if angle <= theta_0 and angle >= 0 and poss_x == 0:
                        print("left wall 1")
                        poss_x = -(2 - dist)
                    elif angle <= np.pi/2 and angle > theta_0:
                        if a >= np.pi/2-0.07 and poss_y == 0:
                            print("bottom wall 1")
                            poss_y = -(2 - dist)                            
                        elif poss_x == 0: 
                            print("left wall 2")
                            poss_x = -(2 - dist)
                    elif angle <= np.pi/2 + theta_0 and angle > np.pi/2 and poss_y == 0:
                        print("bottom wall 2")
                        poss_y = -(2 - dist)
                    elif angle <= np.pi and angle > np.pi/2 + theta_0:
                        if a >= np.pi/2 - 0.07 and poss_x == 0:
                            print("right wall 1")
                            poss_x = 2 - dist
                        elif poss_y == 0: 
                            print("bottom wall 3")
                            poss_y = -(2 - dist)
                    elif angle >= -theta_0 and angle < 0:
                        if a >= np.pi/2-0.07 and poss_y == 0:
                            print("top wall 1")
                            poss_y = 2 - dist
                        elif poss_x == 0: 
                            print("left wall 3")
                            poss_x = -(2 - dist)
                    elif angle >= -np.pi/2 and angle < -theta_0:
                        print("top wall 2")
                        poss_y = 2 - dist
                    elif angle >= -np.pi/2 - theta_0 and angle < -np.pi/2:
                        if a >= np.pi/2-0.07 and poss_y == 0:
                            print("top wall 3")
                            poss_y = 2 - dist
                        elif poss_x == 0: 
                            print("right wall 2")
                            poss_x = 2 - dist
                    elif angle > -np.pi and angle < -np.pi/2 - theta_0 and poss_x == 0:
                        print("right wall 3")
                        poss_x = 2 - dist
                else:
                    # the two sonars detect different walls
                    if angle <= np.pi/2 and angle >= 0 and poss_x == 0:
                        print("L: left wall, FL: top wall")
                        poss_x = -(2 - dist)
                    elif angle <= np.pi and angle > np.pi/2 and poss_y == 0:
                        print("L: bottom wall, FL: left wall")
                        poss_y = -(2 - dist)
                    elif angle < 0 and angle >= -np.pi/2 and poss_y == 0:
                        print("L: top wall, FL: right wall")
                        poss_y = 2 - dist
                    elif angle < -np.pi/2 and angle > -np.pi and poss_x == 0:
                        print("L: right wall, FL: bottom wall")
                        poss_x = 2 - dist
            
            #
            # For the right and the the front right sonars
            #
            if sonar_R < 2 and sonar_FR < 2:
                # we find the angle between the sonar_R and the wall using the robot's orientation and in the R, FR, wall triangle
                # if it is larger than np.pi/2 we keep only angle - np.pi/2
                if angle > 0 and angle < np.pi/2:
                    wallangle = angle
                elif angle > np.pi/2:
                    wallangle = angle - np.pi/2
                elif angle < 0 and angle > -np.pi/2:
                    wallangle = np.pi/2 + angle
                else:
                    wallangle = np.pi + angle

                l_FR = sonar_FR + 0.14142
                l_R = sonar_R + 0.1
                l_wall = sqrt(l_FR**2+l_R**2 - 2*l_FR*l_R*cos(np.pi/4))

                # a is the wall angle that we find  with the sonars in the triangle with edges sonar_L, sonar_FL and the wall edge
                a = np.arccos((l_R**2+l_wall**2-l_FR**2)/(2*l_R*l_wall))

                # print(a, " --- ", wallangle)

                # distance from robot's center to the wall detected by right sonar
                dist = np.sin(wallangle)*(sonar_R + L3)

                # if the two angles are almost the same then the two sonars are detecting the same wall
                if  (wallangle-0.25<a and a < wallangle+0.25) or (wallangle-0.25<a-np.pi/2 and a-np.pi/2 < wallangle+0.25):
                    same_wall = True
                    print("same for FR and R")

                    if angle <= theta_0 and angle >= 0:
                        if a >= np.pi/2-0.07 and poss_x == 0:
                            print("right wall 1")
                            poss_x = 2 - dist
                        elif poss_y == 0: 
                            print("top wall 1")
                            poss_y = 2 - dist
                    elif angle <= np.pi/2 and angle > theta_0 and poss_y == 0:
                        print("top wall 2")
                        poss_y = 2 - dist
                    elif angle <= np.pi/2 + theta_0 and angle > np.pi/2 and angle >= 0:
                        if a >= np.pi/2-0.07 and poss_y == 0:
                            print("top wall 3")
                            poss_y = 2 - dist
                        elif poss_x == 0: 
                            print("left wall 1")
                            poss_x = -(2 - dist)
                    elif angle <= np.pi and angle > np.pi/2 + theta_0 and poss_x == 0:
                        print("left wall 2")
                        poss_x = -(2 - dist)
                    elif angle >= -theta_0 and angle < 0 and poss_x == 0:
                        print("right wall 2")
                        poss_x = 2 - dist
                    elif angle >= -np.pi/2 and angle < -theta_0:
                        if a >= np.pi/2 - 0.07 and poss_y == 0:
                            print("bottom wall 1")
                            poss_y = -(2 - dist)
                        elif poss_x == 0: 
                            print("right wall 3")
                            poss_x = 2 - dist
                    elif angle >= -np.pi/2 - theta_0 and angle < -np.pi/2 and poss_y == 0:
                        print("bottom wall 2")
                        poss_y = -(2 - dist)
                    elif angle > -np.pi and angle < -np.pi/2 - theta_0:
                        if a >= np.pi/2-0.07 and poss_x == 0:
                            print("left wall 3")
                            poss_x = -(2 - dist)
                        elif poss_y == 0: 
                            print("bottom wall 3")
                            poss_y = -(2 - dist)
                else:
                    # the two sonars detect different walls
                    if angle <= np.pi/2 and angle >= 0 and poss_x == 0:
                        print("R: right wall, FR: top wall")
                        poss_x = 2 - dist
                    elif angle <= np.pi and angle > np.pi/2 and poss_y == 0:
                        print("R: top wall, FR: left wall")
                        poss_y = 2 - dist
                    elif angle < 0 and angle >= -np.pi/2 and poss_y == 0:
                        print("R: bottom wall, FR: right wall")
                        poss_y = -(2 - dist)
                    elif angle < -np.pi/2 and angle > -np.pi and poss_x == 0:
                        print("R: left wall, FR: bottom wall")
                        poss_x = 2 - dist
            
            # if sonar_R < 2:
            #     # we find the angle between the sonar_R and the wall using the robot's orientation and in the R, FR, wall triangle
            #     # if it is larger than np.pi/2 we keep only angle - np.pi/2
            #     if angle > 0 and angle < np.pi/2:
            #         wallangle = angle
            #     elif angle > np.pi/2:
            #         wallangle = angle - np.pi/2
            #     elif angle < 0 and angle > -np.pi/2:
            #         wallangle = np.pi/2 + angle
            #     else:
            #         wallangle = np.pi + angle

            #     # using the orientation and the 


            print('Coordinates found by sensors: ', poss_x," and ", poss_y)
            self.x_sensors.append(poss_x)
            self.y_sensors.append(poss_y)
            self.x_sens.publish(poss_x)
            self.y_sens.publish(poss_y)
            #linearization_of_measurement_model
            H = np.transpose(np.array([1,1,1]))

            Cv = np.array([[0.01,0,0],\
                            [0,0.01,0],\
                                [0,0,0.01]])

            #Kalman optimal gain
            K = P*np.transpose(H)*np.linalg.inv((H*P*np.transpose(H)+Cv))

            if poss_x == 0:
                poss_x = 0.01
            if poss_y == 0:
                poss_y = 0.01
            #update
            z = np.array([poss_x, poss_y, poss_y/poss_x]) 
            xbar = np.array([xhat, yhat, thetahat])

            x = xbar + K @ (z - H @ xbar)

            # The P used inside is the P(k+1|k) and the outside the P(k+1)
            P = (np.eye(3) - K*H)*P


            print(x[0],", ",x[1],", ",x[2])

            self.x_positions.append(x[0])
            self.y_positions.append(x[1])
            self.theta_angle.append(x[2])
            self.x_pos.publish(x[0])
            self.y_pos.publish(x[1])
            self.theta_ang.publish(x[2])

            self.pub_rate.sleep()

    def turn_off(self):
        # print("Turned off")
        t=np.arange(0,len(self.x_positions))
        fig , ax = plt.subplots(figsize=(12,12),nrows=1,ncols=3)
        fig.suptitle("State of Robot") 
        ax[0].set(title=" X position")
        ax[1].set(title=" Y position")
        ax[2].set(title=" Theta angle")
        ax[0].plot(t,self.x_positions)
        ax[1].plot(t,self.y_positions)
        ax[2].plot(t,self.theta_angle)

        plt.savefig('state')  


        fig1, ax1 = plt.subplots(figsize=(12, 12), nrows=1, ncols=2)
        fig1.suptitle("Positions found by sonars")
        ax1[0].set(title=" X position")
        ax1[1].set(title=" Y position")
        ax1[0].plot(t,self.x_sensors)
        ax1[1].plot(t,self.y_sensors)
        plt.savefig('Positions')

        pass

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    follower = mymobibot_localization(rate)
    rospy.on_shutdown(follower.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException:
        pass
