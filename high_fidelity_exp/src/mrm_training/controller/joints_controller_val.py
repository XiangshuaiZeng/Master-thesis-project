#!/usr/bin/env python

import rospy
from pid_controller import PID
import numpy as np
from std_msgs.msg import Bool, Float64MultiArray
from gazebo_msgs.srv import ApplyJointEffort
from sensor_msgs.msg import JointState

class joints_controller:
    def __init__(self):
        # initilize controllers
        self.position_controllers  = []
        self.ts = rospy.get_param('ts')

        args = rospy.get_param("/position_controllers")

        self.joint_01_controller = PID(args["joint_01"], self.ts)
        self.joint_02_controller = PID(args["joint_02"], self.ts)
        self.joint_03_controller = PID(args["joint_03"], self.ts)
        self.prismatic_controller = PID(args["prismatic"], self.ts)
        self.joint_01_2_controller = PID(args["joint_01_2"], self.ts)
        self.joint_02_2_controller = PID(args["joint_02_2"], self.ts)
        self.joint_03_2_controller = PID(args["joint_03_2"], self.ts)

        self.position_controllers.append(self.joint_01_controller)
        self.position_controllers.append(self.joint_02_controller)
        self.position_controllers.append(self.joint_03_controller)
        self.position_controllers.append(self.prismatic_controller)
        self.position_controllers.append(self.joint_01_2_controller)
        self.position_controllers.append(self.joint_02_2_controller)
        self.position_controllers.append(self.joint_03_2_controller)

        self.reset_controller = True
        self.pause_sim = True
        self.set_points = [0,0,0,0,0,0,0]

        # service for applying joint efforts to gazebo
        self.torque_srv = rospy.ServiceProxy("gazebo/apply_joint_effort", ApplyJointEffort)

        # create a subscriber to subscribe the set_points from the envirnment
        rospy.Subscriber('/position_controllers/command', Float64MultiArray, self.joint_targets_callback)

        # subscribe the joint states
        rospy.Subscriber('/joint_states', JointState, self.joints_state_callback)

        # param for reset the controllers
        rospy.Subscriber('/reset_controller', Bool, self.reset_controller_callback)

        # param for stop publishing efforts
        rospy.Subscriber('/pause_sim', Bool, self.pause_sim_callback)

        self.joints_name = ['joint_01', 'joint_02', 'joint_03', 'prismatic', 'joint_01_2', 'joint_02_2', 'joint_03_2']

    def joint_targets_callback(self, msg):
        self.set_points = msg.data
        rospy.loginfo(self.set_points)

    def joints_state_callback(self,msg):
        self.current_pos = msg.position

    def reset_controller_callback(self, msg):
        self.reset_controller = msg.data

    def pause_sim_callback(self, msg):
        self.pause_sim = msg.data

    def check_all_systems_ready(self):
        """
        We check that all systems are ready
        :return:
        """
        # collect info on joint states
        joint_states_msg = None
        while joint_states_msg is None and not rospy.is_shutdown():
            joint_states_msg = rospy.wait_for_message("/joint_states", JointState, timeout=0.1)
            self.current_pos = joint_states_msg

        rospy.loginfo("ALL SYSTEMS READY")


    def update(self):
        if self.reset_controller:
            for i in range(7):
                self.position_controllers[i].clear()
        else:
            if not self.pause_sim:
                effort_array = [0, 0, 0, 0, 0, 0, 0]
                for i in range(7):
                    effort_array[i] = self.position_controllers[i].update(self.set_points[i] - self.current_pos[i])

                duration_time = self.ts
                start_time = rospy.Time(0.0)
                duration = rospy.Time(duration_time)
                i = 0
                for joint_name in self.joints_name:
                    self.torque_srv(joint_name, effort_array[i], start_time, duration)
                    i += 1

rospy.init_node('joints_controller')
rate = rospy.Rate(100) # rate at 100 hz
mrm_controller = joints_controller()

mrm_controller.check_all_systems_ready()

while not rospy.is_shutdown():
    mrm_controller.update()
    rate.sleep()
