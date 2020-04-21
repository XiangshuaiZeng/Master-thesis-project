#!/usr/bin/env python

import rospy
import math
import random
from std_msgs.msg import String
from std_msgs.msg import Float64
from gazebo_msgs.srv import ApplyJointEffort, SetModelConfiguration

class JointPub(object):   # class for publishing the torque commands for all the joints
    def __init__(self):
        rospy.loginfo("Wait for services")
        rospy.wait_for_service("gazebo/apply_joint_effort")
        rospy.wait_for_service("gazebo/set_model_configuration")  # for setting joints back to initial
        rospy.loginfo("Got it.")
        self.torque_srv = rospy.ServiceProxy("gazebo/apply_joint_effort", ApplyJointEffort)
        self.set_joints_srv = rospy.ServiceProxy("gazebo/set_model_configuration", SetModelConfiguration)

        self.init_pos = [-0.45, 1.1, 0.8, 0]

        self.joints_name = ["joint_01", "joint_02", "joint_03", "prismatic"]

    def set_init_condition(self):
        """
        Sets joints to initial position 
        :return:
        """
        joint1_noise = self.init_pos[0] + random.uniform(0.0,0.1)
        joint2_noise = self.init_pos[1] + random.uniform(-0.1,0.05)
        joint3_noise = self.init_pos[2] + random.uniform(-0.1,0.1)
        prismatic_noise = self.init_pos[3] + random.uniform(0.0,0.04)

        initial_pose = [joint1_noise, joint2_noise, joint3_noise, prismatic_noise]

        self.set_joints_srv('mrm', 'robot_description', self.joints_name, initial_pose)


    def apply_joints_effort(self, effort_array, duration_time):
        start_time = rospy.Time(0.0)
        duration = rospy.Time(duration_time)
        i = 0
        for joint_name in self.joints_name:
            self.torque_srv(joint_name, effort_array[i], start_time, duration)
            i += 1

    # function for testing
    def start_sinus_loop(self, rate_value = 1):
        rospy.logdebug("Start Loop")
        w = 0.0
        x = 2.0
        rate = rospy.Rate(rate_value)

        pos_x = [5*x, 2*x, x, 2*x]
        self.apply_joints_effort(pos_x, 0.1)
        x = -x
        rate.sleep()
        self.set_init_condition()



if __name__=="__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 0.2
    #joint_publisher.start_loop(rate_value)
    joint_publisher.start_sinus_loop(rate_value)  #
