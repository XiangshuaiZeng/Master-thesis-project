#!/usr/bin/env python

import rospy
import math
import random
from std_msgs.msg import String, Float64, Float64MultiArray
from gazebo_msgs.srv import SetModelConfiguration

class JointPub(object):   # class for publishing the commands for all the joints
    def __init__(self):

        self.position_pub = rospy.Publisher('/position_controllers/command', Float64MultiArray, queue_size=1)

        self.init_pos = [0.51, -1.27,-0.95, 0] # for 110mm pipe

        rospy.loginfo("Wait for services")
        rospy.wait_for_service("gazebo/set_model_configuration")  # for setting joints back to initial
        rospy.loginfo("Got it.")
        self.set_joints_srv = rospy.ServiceProxy("gazebo/set_model_configuration", SetModelConfiguration)
        self.joints_name = ["joint_01", "joint_02", "joint_03", "prismatic", "joint_01_2", "joint_02_2", "joint_03_2"]

    def set_init_condition(self):

        self.check_publishers_connection()

        joint1_noise = self.init_pos[0]
        joint2_noise = self.init_pos[1]
        joint3_noise = self.init_pos[2]
        prismatic_noise = self.init_pos[3]

        initial_pose = [joint1_noise, joint2_noise, joint3_noise, prismatic_noise, joint1_noise, joint2_noise, -joint3_noise]

        self.set_joints_srv('mrm', 'robot_description', self.joints_name, initial_pose)

    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(50)  # 50hz
        while (self.position_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to position_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("position_pub Publisher Connected")

    def move_joints(self, joints_array):
        joint_values = Float64MultiArray()
        joint_values.data = joints_array
        joint_values.layout.dim = []
        joint_values.layout.data_offset = 0

        self.position_pub.publish(joint_values)

if __name__=="__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 50.0
    joint_publisher.start_sinus_loop(rate_value)  #
