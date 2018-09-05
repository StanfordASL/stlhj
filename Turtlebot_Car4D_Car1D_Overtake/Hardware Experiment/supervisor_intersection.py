#!/usr/bin/env python

import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Int16, Bool, Float64MultiArray
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, TransformStamped
from asl_turtlebot.msg import StateSpace
import tf
import math
from enum import Enum

# threshold at which we consider the robot at a location
POS_EPS = .05
THETA_EPS = .3

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    NAV = 3
    MANUAL = 4

class Supervisor:
    """ the state machine of the turtlebot """

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)

        # current pose
        self.x_joe = 0
        self.y_joe = 0
        self.theta_joe = 0

        self.x_mo = 0
        self.y_mo = 0
        self.theta_mo = 0

        # Pose Goal
        self.x_g_joe = 0.2#-0.05 #m
        self.y_g_joe = -0.45 #m #-0.55 no pass, #-0.4 pass
        self.theta_g_joe = np.pi/2 #rad

        self.x_g_mo = -0.2
        self.y_g_mo = -0.4#0.45
        self.theta_g_mo = -np.pi/2

        self.sim_start = 0.
        self.accel = 0.
        self.omega = 0.
        self.cmd_vel = 0.
        self.dt = 0.1
        self.slowtime = 1.

        # current mode
        self.mode = Mode.IDLE

        self.nav_goal_publisher_mo = rospy.Publisher('/cmd_nav_mo', Pose2D, queue_size=1)
        #self.nav_goal_publisher_joe = rospy.Publisher('/cmd_nav_joe', Pose2D, queue_size=1)

        self.cmd_vel_publisher_mo = rospy.Publisher('/cmd_vel_mo', Twist, queue_size=1)
        self.cmd_vel_publisher_joe = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.pose_goal_publisher_joe = rospy.Publisher('/cmd_pose_joe', Pose2D, queue_size=1)
        self.pose_goal_publisher_mo = rospy.Publisher('/cmd_pose_mo', Pose2D, queue_size=1)

        self.pose_publisher_joe = rospy.Publisher('/pose_joe', Pose2D, queue_size=1)
        self.pose_publisher_mo = rospy.Publisher('/pose_mo', Pose2D, queue_size=1)

        self.StateSpace_publisher = rospy.Publisher('/StateSpace',Float64MultiArray, queue_size=1 )

        #Vicon Position
        rospy.Subscriber('/vicon/joe_burger/joe_burger', TransformStamped, self.joe_burger_callback)
        rospy.Subscriber('/vicon/mo_burger/mo_burger', TransformStamped, self.mo_burger_callback)

        #keyboard teleop functionalities
        rospy.Subscriber('/cmd_teleop_vel', Twist, self.teleop_vel_callback)
        rospy.Subscriber('/cmd_teleop_mode', Int16, self.teleop_mode_callback)

        #listen for MATLAB
        rospy.Subscriber('/ctrl_MATLAB',Float64MultiArray, self.MATLAB_callback)

    def joe_burger_callback(self, msg):

        self.x_joe = msg.transform.translation.x
        self.y_joe = msg.transform.translation.y
        euler = tf.transformations.euler_from_quaternion([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        temp_theta_joe = euler[2]
        self.theta_joe = np.where(temp_theta_joe<0 , 2*np.pi+temp_theta_joe, temp_theta_joe)

        pose_msg_joe = Pose2D()
        pose_msg_joe.x = self.x_joe
        pose_msg_joe.y = self.y_joe
        pose_msg_joe.theta = self.theta_joe
        self.pose_publisher_joe.publish(pose_msg_joe)

    def mo_burger_callback(self, msg):

        self.x_mo = msg.transform.translation.x
        self.y_mo = msg.transform.translation.y
        euler = tf.transformations.euler_from_quaternion([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        self.theta_mo = euler[2]

        pose_msg_mo = Pose2D()
        pose_msg_mo.x = self.x_mo
        pose_msg_mo.y = self.y_mo
        pose_msg_mo.theta = self.theta_mo
        self.pose_publisher_mo.publish(pose_msg_mo)

    def teleop_vel_callback(self, msg):
        self.mode = Mode.MANUAL
        self.cmd_vel_publisher_mo.publish(msg)

    # MODIFIED
    def teleop_mode_callback(self, msg):
        for mode in Mode:
            if mode.value == msg.data:
                self.mode = mode

    def go_to_pose_joe(self):
        """ sends the current desired pose to the pose controller """
        pose_g_msg_joe = Pose2D()
        pose_g_msg_joe.x = self.x_g_joe
        pose_g_msg_joe.y = self.y_g_joe
        pose_g_msg_joe.theta = self.theta_g_joe
        self.pose_goal_publisher_joe.publish(pose_g_msg_joe)

    def go_to_pose_mo(self):
        """ sends the current desired pose to the pose controller """
        pose_g_msg_mo = Pose2D()
        pose_g_msg_mo.x = self.x_g_mo
        pose_g_msg_mo.y = self.y_g_mo
        pose_g_msg_mo.theta = self.theta_g_mo
        self.pose_goal_publisher_mo.publish(pose_g_msg_mo)

    def MATLAB_callback(self,msg):
        self.accel = msg.data[0] #Acceleration
        self.omega = msg.data[1] #Angular Velocity

    def MATLAB_forward(self):
        StateSpace = Float64MultiArray()
        if self.mode == Mode.IDLE:
            StateSpace.data = [self.x_joe,self.y_joe,self.theta_joe,self.cmd_vel,self.y_mo,0.]
        else:

            if self.y_mo < -0.5:
                y_mo = -0.5
            else:
                y_mo = self.y_mo
            StateSpace.data = [self.x_joe,self.y_joe,self.theta_joe,self.cmd_vel*self.slowtime,y_mo, (rospy.get_time()-self.sim_start)/self.slowtime]
        self.StateSpace_publisher.publish(StateSpace)

    def nav_joe(self):
        """ sends the current desired pose to the pose controller """
        if self.omega*self.dt/self.slowtime+self.theta_joe > -0.4 and self.omega*self.dt/self.slowtime+self.theta_joe < 0.:
            print("exceeding angle")
            self.cmd_ang = 0.
        else:
            self.cmd_ang = self.omega/self.slowtime

        if self.accel*self.dt/self.slowtime+self.cmd_vel > 0.15/self.slowtime or self.accel*self.dt/self.slowtime+self.cmd_vel < 0.:
            #print("exceeding velocity")
            #print(self.cmd_vel)s
            pass
        else:
            #print((rospy.get_time()-self.sim_start))/self.slowtime
            self.cmd_vel = (self.accel*self.dt/self.slowtime + self.cmd_vel)#*(rospy.get_time()-self.timer)

        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_vel
        cmd_msg.angular.z = self.cmd_ang
        self.cmd_vel_publisher_joe.publish(cmd_msg)

    def nav_mo(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg_mo = Pose2D()
        pose_g_msg_mo.x = self.x_g_mo
        pose_g_msg_mo.y = -1.
        pose_g_msg_mo.theta = self.theta_g_mo
        self.nav_goal_publisher_mo.publish(pose_g_msg_mo)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher_mo.publish(vel_g_msg)
        self.cmd_vel_publisher_joe.publish(vel_g_msg)

    def close_to(self,x_g,y_g,theta_g,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-x_g)<POS_EPS and abs(y-y_g)<POS_EPS and abs(theta-theta_g)<THETA_EPS)

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            #print("Staying still...")
            self.sim_start = 0
            #self.MATLAB_forward()
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            #print(self.y_g_joe-self.y_joe)
            print("On the move!")
            vel_g_msg = Twist()
            if self.close_to(self.x_g_joe,self.y_g_joe,self.theta_g_joe,self.x_joe,self.y_joe,self.theta_joe) and self.close_to(self.x_g_mo,self.y_g_mo,self.theta_g_mo,self.x_mo,self.y_mo,self.theta_mo):
                print("I'm close!")
                self.mode = Mode.IDLE
                return

            if self.close_to(self.x_g_joe,self.y_g_joe,self.theta_g_joe,self.x_joe,self.y_joe,self.theta_joe):
                #print("I'm close to joe!")
                self.cmd_vel_publisher_joe.publish(vel_g_msg)
            else:
                self.go_to_pose_joe()

            if self.close_to(self.x_g_mo,self.y_g_mo,self.theta_g_mo,self.x_mo,self.y_mo,self.theta_mo):
                #print("I'm close to mo!")
                self.cmd_vel_publisher_mo.publish(vel_g_msg)
            else:
                self.go_to_pose_mo()

        elif self.mode == Mode.NAV:
            print("navigating!")
            if self.sim_start == 0:
                self.cmd_vel = 0.#0.15/2
                self.sim_start = rospy.get_time()
            if (rospy.get_time()-self.sim_start)/self.slowtime>12.:
                self.mode = Mode.IDLE
                return

            if self.close_to(-0.6,0.2,np.pi,self.x_joe,self.y_joe,self.theta_joe):
                #print("Joe: I'm at the border!")
                self.mode = Mode.IDLE
            else:
                #self.timer = rospy.get_time()
                #print(rospy.get_time()-self.sim_start)
                self.MATLAB_forward()
                #print("moving!")
                self.nav_joe()

            if self.close_to(-0.2,-1,-np.pi/2.,self.x_mo,self.y_mo,self.theta_mo):
                #print("Mo: I'm here!")
                vel_g_msg = Twist()
                self.cmd_vel_publisher_mo.publish(vel_g_msg)
            else:
                self.nav_mo()

        elif self.mode == Mode.MANUAL:
            pass

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
