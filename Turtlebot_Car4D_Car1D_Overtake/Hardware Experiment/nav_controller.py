#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, TransformStamped
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi

# control gains
K1 = 0.4
K2 = 0.8
K3 = 0.8

# maximum velocity
V_MAX = 0.25

# maximim angular velocity
W_MAX = 2

class PoseController:

    def __init__(self):
        rospy.init_node('turtlebot_nav_controller', anonymous=True)
        self.cmd_vel_publisher_mo = rospy.Publisher('/cmd_vel_mo', Twist, queue_size=10)
        self.cmd_vel_publisher_benoit = rospy.Publisher('/cmd_vel_benoit', Twist, queue_size=10)

        # current state
        self.x_mo = 0
        self.y_mo = 0
        self.theta_mo = 0

        self.x_benoit = 0
        self.y_benoit = 0
        self.theta_benoit = 0

        # goal state
        self.x_g_mo = -0.2
        self.y_g_mo = -0.8
        self.theta_g_mo = -np.pi/2
        # self.x_g_mo = 0.2
        # self.y_g_mo = 0.8
        # self.theta_g_mo = np.pi/2

        self.x_g_benoit = 0.2 #m
        self.y_g_benoit = 1. #m
        self.theta_g_benoit = np.pi/2 #rad

        self.cmd_vel_mo = 0.05   #0.05#0.075 #first one #m/s
        self.cmd_vel_benoit = 0.01
        rospy.Subscriber('/cmd_nav_mo', Pose2D, self.cmd_pose_callback_mo)
        rospy.Subscriber('/pose_mo', Pose2D, self.mo_burger_callback)

        rospy.Subscriber('/cmd_nav_benoit', Pose2D, self.cmd_pose_callback_benoit)
        rospy.Subscriber('/pose_benoit', Pose2D, self.benoit_burger_callback)


    def mo_burger_callback(self, msg):

        self.x_mo = msg.x
        self.y_mo = msg.y
        self.theta_mo = msg.theta

    def cmd_pose_callback_mo(self, data):
        self.x_g_mo = data.x
        self.y_g_mo = data.y
        self.theta_g_mo = data.theta
        self.run_pose_controller_mo()

    def benoit_burger_callback(self, msg):

        self.x_benoit = msg.x
        self.y_benoit = msg.y
        self.theta_benoit = msg.theta

    def cmd_pose_callback_benoit(self, data):
        self.x_g_benoit = data.x
        self.y_g_benoit = data.y
        self.theta_g_benoit = data.theta
        self.run_pose_controller_benoit()

    def run_pose_controller_mo(self):
        """ runs a simple feedback pose controller """

        rel_coords = np.array([self.x_mo-self.x_g_mo, self.y_mo-self.y_g_mo])
        R = np.array([[np.cos(self.theta_g_mo), np.sin(self.theta_g_mo)], [-np.sin(self.theta_g_mo), np.cos(self.theta_g_mo)]])
        rel_coords_rot = np.dot(R,rel_coords)

        th_rot = self.theta_mo-self.theta_g_mo
        rho = linalg.norm(rel_coords)
        ang = np.arctan2(rel_coords_rot[1],rel_coords_rot[0])+np.pi
        angs = wrapToPi(np.array([ang-th_rot, ang]))
        alpha = angs[0]
        delta = angs[1]

        V = K1*rho*np.cos(alpha)
        om = K2*alpha + K1*np.sinc(2*alpha/np.pi)*(alpha+K3*delta)

        # Apply saturation limits
        cmd_x_dot = np.sign(V)*min(V_MAX, np.abs(V))
        cmd_theta_dot = np.sign(om)*min(W_MAX, np.abs(om))

        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_vel_mo
        cmd_msg.angular.z = cmd_theta_dot
        self.cmd_vel_publisher_mo.publish(cmd_msg)

    def run_pose_controller_benoit(self):
        """ runs a simple feedback pose controller """

        rel_coords = np.array([self.x_benoit-self.x_g_benoit, self.y_benoit-self.y_g_benoit])
        R = np.array([[np.cos(self.theta_g_benoit), np.sin(self.theta_g_benoit)], [-np.sin(self.theta_g_benoit), np.cos(self.theta_g_benoit)]])
        rel_coords_rot = np.dot(R,rel_coords)

        th_rot = self.theta_benoit-self.theta_g_benoit
        rho = linalg.norm(rel_coords)
        ang = np.arctan2(rel_coords_rot[1],rel_coords_rot[0])+np.pi
        angs = wrapToPi(np.array([ang-th_rot, ang]))
        alpha = angs[0]
        delta = angs[1]

        V = K1*rho*np.cos(alpha)
        om = K2*alpha + K1*np.sinc(2*alpha/np.pi)*(alpha+K3*delta)

        # Apply saturation limits
        cmd_x_dot = np.sign(V)*min(V_MAX, np.abs(V))
        cmd_theta_dot =0# np.sign(om)*min(W_MAX, np.abs(om))

        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_vel_benoit
        cmd_msg.angular.z = cmd_theta_dot
        self.cmd_vel_publisher_benoit.publish(cmd_msg)




if __name__ == '__main__':
    pctrl = PoseController()
    rospy.spin()
