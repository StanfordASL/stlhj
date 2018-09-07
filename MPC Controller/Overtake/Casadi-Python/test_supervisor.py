import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Int16, Bool, Float64MultiArray
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, TransformStamped
from asl_turtlebot.msg import StateSpace

class Supervisor:
    """ the state machine of the turtlebot """

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        # current pose
        self.x_joe = -0.2
        self.y_joe = 0.
        self.y_mo = 0.5
        self.theta_joe = np.pi/2
        self.sim_start = 0.


        self.StateSpace_publisher = rospy.Publisher('/StateSpace',Float64MultiArray, queue_size=1 )

        #listen for MATLAB
        rospy.Subscriber('/ctrl_MATLAB',Float64MultiArray, self.MATLAB_callback)

    def MATLAB_callback(self,msg):
        print(msg.data)
        

    def loop(self):

        StateSpace = Float64MultiArray()
        StateSpace.data = [self.x_joe,self.y_joe,self.theta_joe,0.,self.y_mo,19.]
        self.StateSpace_publisher.publish(StateSpace)
        self.sim_start = rospy.get_time()
    def run(self):
        rate = rospy.Rate(0.5) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
