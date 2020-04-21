#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Point, Quaternion, Vector3, Transform
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64
from math import *
from mrm_training_force.msg import Floatarray
import numpy as np
import math

class MrmState(object):

    def __init__(self, weight_r1=1.0, weight_r2=1.0, weight_r3=1.0):
        rospy.logdebug("Starting MrmState Class object...")
        self.desired_target_point = Point(1.0, 1.0, 1.0)     # note that the data type should be Point()

        self._weight_r1 = weight_r1
        self._weight_r2 = weight_r2
        self._weight_r3 = weight_r3

        self.joints_state = JointState()
        self.end_effector_pos = Point()

        self.collision_reward = rospy.get_param("/collision_reward") # if collision happens, give a very low reward
        self.reach_lim_reward = rospy.get_param("/reach_lim_reward") # if joints limits are reached, give a very low reward

        # get the limit info for each joint
        self.prismatic_lim = rospy.get_param("/prismatic_lim")
        self.joint_01_lim = rospy.get_param("/joint_01_lim")
        self.joint_02_lim = rospy.get_param("/joint_02_lim")
        self.joint_03_lim = rospy.get_param("/joint_03_lim")

        # We use it to get the joints positions and calculate the reward associated to it
        rospy.Subscriber("/joint_states", JointState, self.joints_state_callback)

        # Get the position of end-effector
        rospy.Subscriber("/end_effector_pos", Transform, self.end_effector_pos_callback)

        # Get the data from laser scanner
        rospy.Subscriber("/laser_scan", LaserScan, self.laser_scan_callback)

        # Get action
        rospy.Subscriber("/action", Floatarray, self.action_callback)

        # data from the previous time step including laser data, joint position at t-1
        self.history = []

    def check_all_systems_ready(self):
        """
        We check that all systems are ready
        :return:
        """
        # collect info on joint states
        joint_states_msg = None
        while joint_states_msg is None and not rospy.is_shutdown():
            try:
                joint_states_msg = rospy.wait_for_message("/joint_states", JointState, timeout=0.1)
                self.joints_state = joint_states_msg
                rospy.logdebug("Current joint_states READY")
            except Exception as e:
                rospy.logdebug("Current joint_states not ready yet, retrying==>"+str(e))

        # collect position of end-effector
        end_pos_msg = None
        while end_pos_msg is None and not rospy.is_shutdown():
            try:
                end_pos_msg = rospy.wait_for_message("/end_effector_pos", Transform, timeout=0.1)
                self.end_effector_pos = end_pos_msg
                rospy.logdebug("End-effector position READY")
            except Exception as e:
                rospy.logdebug("End-effector position not ready yet, retrying==>"+str(e))

        # collect laser data
        laser_msg = None
        while laser_msg is None and not rospy.is_shutdown():
            try:
                laser_msg = rospy.wait_for_message("/laser_scan", LaserScan, timeout=0.1)
                self.laser_data = list(laser_msg.ranges)
                rospy.logdebug("Laser scanner READY")
            except Exception as e:
                rospy.logdebug("Laser scanner not ready yet, retrying==>"+str(e))

        rospy.logdebug("ALL SYSTEMS READY")

    def set_desired_target_point(self, x, y, z):
        """
        Target point location
        :return:
        """
        self.desired_target_point.x = x
        self.desired_target_point.y = y
        self.desired_target_point.z = z

    def get_joint_states(self):
        return self.joints_state

    def get_laser_data(self):
        laser_d = self.laser_data
        for i in range(len(laser_d)):
            if laser_d[i] == float('inf'):
                laser_d[i] = 0.5
            elif laser_d[i] <= 0.01:
                laser_d[i] = 0.01
        return laser_d

    def joints_state_callback(self,msg):
        self.joints_state = msg

    def end_effector_pos_callback(self, msg):
        self.end_effector_pos = msg

    def laser_scan_callback(self, msg):
        self.laser_data = list(msg.ranges)

    def action_callback(self, msg):
        self.action = msg.data

    def get_distance_from_point(self, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        end_pos = self.end_effector_pos.translation
        a = np.array((end_pos.x, end_pos.y))
        b = np.array((p_end.x, p_end.y))
        distance = b - a
        return distance

    def calculate_reward_distance_from_des_point(self, weight=1.0):
        """
        We calculate the distance from the desired point.
        The closser the better
        :param weight:
        :return:reward
        """
        distance = np.linalg.norm(self.get_distance_from_point(self.desired_target_point))

        # ln loss
        reward = -np.log(50*distance+0.1) + np.log(0.8)

        reward = weight * reward
        return reward

    def calculate_reward_distance_from_obstacle(self, weight = 1.0):
        """
        Calculate the reward based on the distance from the end-effector to the
        nearest pipe; the closer of the distance, the less the reward;
        Use information from the sensor at the end-effector
        """
        laser_data = self.get_laser_data()
        obstacle_distance = min(laser_data) # np.asarray(laser_data[3:7])   # the middle 4 data point
        # obstacle_distance = np.asarray(laser_data[3:7])

        offset = 0.005  # offset to prevent 1 / 0.0 occurs, resulting a inf reward
        beta = 5.0
        reward = -1.0 / (beta * obstacle_distance + offset)
        reward = np.sum(reward) * weight
        # reward *= weight

        # if obstacle_distance <= 0.01:
        #     reward = self.collision_reward
        # else:
        #     reward = 0.0
        return reward

    def calculate_reward_joint_change(self, weight=0.001):
        """
        We calculate reward based on the position change of the joints
        :return:
        """
        prev_joint_pos = self.history[0:4]
        joint_states = self.get_joint_states()
        curr_joint_pos = joint_states.position[0:4]

        # calculate the position change
        pos_change = np.linalg.norm(np.asarray(curr_joint_pos) - np.asarray(prev_joint_pos))

        # compute the reward
        offset = 0.005  # offset to prevent 1 / 0.0 occurs, resulting a inf reward
        beta = 5.0
        reward = -1.0 / (beta * pos_change + offset)
        reward = reward * weight

        return reward


    def calculate_total_reward(self):
        r1 = self.calculate_reward_distance_from_des_point(self._weight_r1)
        r2 = self.calculate_reward_joint_change(self._weight_r2)
        r3 = self.calculate_reward_distance_from_obstacle(self._weight_r3)

        total_reward = r1 + r3 + r2

        # rospy.loginfo("r1 distance from target=" + str(r1))
        # rospy.loginfo("r2 joint_change=" + str(r2))
        # rospy.loginfo("r3 distance from pipe=" + str(r3) + '\n')

        return total_reward

    def get_observations(self):
        """
        Returns the state of the robot needed for RL Algorithm
        The state will be defined by an array of the:

        states = ["prismatic_joint_states"
                 "joint_01_states",
                 "joint_02_states",
                 "joint_03_states",
                 "end_effector_pos",
                 "target_pos",
                 "laser_scanner_data"]

        :return: observation
        """
        end_pos = [self.end_effector_pos.translation.x, self.end_effector_pos.translation.y]
        target_pos = [self.desired_target_point.x, self.desired_target_point.y]

        joint_states = self.get_joint_states()
        joint_pos = list(joint_states.position)
        joint_vel = list(joint_states.velocity)
        laser_data = self.get_laser_data()

        present = joint_pos + joint_vel + laser_data
        # in the first step, the history is set to be the same as the present
        if len(self.history) == 0:
            self.history = present

        observation = present + self.history + end_pos + target_pos
        # record the current joint pos and laser data
        self.history = present

        return observation

    def if_reach_target(self):
        distance_from_desired_point = self.get_distance_from_point(self.desired_target_point)
        distance_norm = np.linalg.norm(distance_from_desired_point)
        if distance_norm < 0.04:
            rospy.loginfo('Target is reached!')
            return True
        else:
            return False

    def if_collision(self):
        min_laser_dis = min(self.get_laser_data())
        if min_laser_dis < 0.02:
            return True
        else:
            return False

    def if_reach_limit(self):
        """
        Detect whether either of the joints reach their physical limits, if so,
        end the episode.
        """
        joint_states = self.get_joint_states()
        joint_01_pos = joint_states.position[0]
        joint_02_pos = joint_states.position[1]
        joint_03_pos = joint_states.position[2]
        prismatic_joint_pos = joint_states.position[3]

        if (self.prismatic_lim[0] < prismatic_joint_pos < self.prismatic_lim[1]) and (self.joint_01_lim[0] < joint_01_pos < self.joint_01_lim[1]) and \
            (self.joint_02_lim[0] < joint_02_pos < self.joint_02_lim[1]) and (self.joint_03_lim[0] < joint_03_pos < self.joint_03_lim[1]):
            return False
        else:
            return True

    def process_data(self):
        """
        We return the total reward based on the state in which we are in and if its done or not
        :return: reward, done
        """
        collide = self.if_collision()
        reach_lim = self.if_reach_limit()
        reach_target = self.if_reach_target()
        done = reach_target or reach_lim

        if reach_lim:
            # rospy.loginfo("Joints reach limits, give it a very low reward")
            total_reward = self.reach_lim_reward
            return total_reward, reach_target , done

        total_reward = self.calculate_total_reward()

        return total_reward, reach_target , done


    def testing_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.calculate_total_reward()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('mrm_state_node', anonymous=True)
    monoped_state = MrmState()
    monoped_state.testing_loop()
