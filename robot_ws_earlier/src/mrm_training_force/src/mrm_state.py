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
            elif laser_d[i] < 0.01:
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
        obstacle_distance = min(self.get_laser_data())
        offset = 0.005  # offset to prevent 1 / 0.0 occurs, resulting a inf reward
        beta = 5.0
        reward = -1.0 / (beta * obstacle_distance + offset)
        reward = reward * weight
        # if obstacle_distance <= 0.01:
        #     reward = self.collision_reward
        # else:
        #     reward = 0.0
        return reward

    def calculate_reward_joint_effort(self, weight=0.001):
        """
        We calculate reward based on the joints effort readings. The more near 0 the better.
        :return:
        """
        acumulated_joint_effort = 0.0
        joint_efforts = self.action
        for joint_effort in joint_efforts:
            # Abs to remove sign influence, it doesnt matter the direction of the effort.
            acumulated_joint_effort += abs(joint_effort)

        reward = -weight * acumulated_joint_effort
        rospy.logdebug("calculate_reward_joint_effort>>reward=" + str(reward))
        return reward

    def calculate_total_reward(self):
        r1 = self.calculate_reward_distance_from_des_point(self._weight_r1)
        #r2 = self.calculate_reward_joint_effort(self._weight_r2)
        r3 = self.calculate_reward_distance_from_obstacle(self._weight_r3)
        # r4 = self.calculate_reward_joint_vel()

        total_reward = r1 + r3 #+ r4  #+ r2

        # rospy.loginfo("r1 distance from target=" + str(r1))
        # rospy.loginfo("r3 distance from pipe=" + str(r3) + '\n')
        # rospy.loginfo("r2 joint_effort=" + str(r2))
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
        end_pos = np.array((self.end_effector_pos.translation.x, self.end_effector_pos.translation.y))
        target_pos = np.array((self.desired_target_point.x, self.desired_target_point.y))

        joint_states = self.get_joint_states()
        joint_01_pos = joint_states.position[0]
        joint_02_pos = joint_states.position[1]
        joint_03_pos = joint_states.position[2]
        prismatic_joint_pos = joint_states.position[3]

        joint_01_vel = joint_states.velocity[0]
        joint_02_vel = joint_states.velocity[1]
        joint_03_vel = joint_states.velocity[2]
        prismatic_joint_vel = joint_states.velocity[3]

        laser_data = self.get_laser_data()
        observation = [joint_01_pos, joint_02_pos, joint_03_pos, prismatic_joint_pos, joint_01_vel, joint_02_vel, joint_03_vel, prismatic_joint_vel]
        observation.extend(end_pos)
        observation.extend(target_pos)
        observation.extend(laser_data)
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
        reach_lim = self.if_reach_limit()
        reach_target = self.if_reach_target()
        done = reach_target or reach_lim

        if reach_lim:
            rospy.loginfo("Joints reach limits, give it a very low reward")
            total_reward = self.reach_lim_reward
            return total_reward, reach_target, done

        total_reward = self.calculate_total_reward()

        return total_reward, reach_target , done

    def change_weights(self, weight_r1, weight_r2, weight_r3):
        self._weight_r1 = weight_r1
        self._weight_r2 = weight_r2
        self._weight_r3 = weight_r3

    def calculate_pipe_map(self):
        end_pos_msg = self.end_effector_pos
        laser_msg = self.get_laser_data()
        pipe_points = []

        #calculate the coordinate of obstacle under end_effector frame
        for i in range(len(laser_msg) + 1):
            delta_t = pi / len(lasor_msg)
            if laser_msg[i] < 1.0:
                XY = [laser_msg[i] * cos(delta_t * i), laser_msg[i] * sin(delta_t * i)]
                pipe_points.append(XY)

        # get the orientation matrix


    def testing_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.calculate_total_reward()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('mrm_state_node', anonymous=True)
    monoped_state = MrmState()
    monoped_state.testing_loop()
