#!/usr/bin/env python

import rospy
from math import *
from  dynamixel_workbench_msgs.srv import DynamixelCommand

rospy.init_node('sin_path')
rate = rospy.Rate(10)

command_srv = rospy.ServiceProxy("/dynamixel_workbench/dynamixel_command", DynamixelCommand)

rotation_id = [32,33,34]
translation_id = 35

ratio = 300.0 / 1024
zero_point = 512


A = 20 # 30 degrees

t = 0

torque = 800
target1 = 1024 + torque
target2 = torque

while not rospy.is_shutdown():
	# move the rotational joints
	target = A * sin(2*pi/50 * t) # in degree
	target = target / ratio + zero_point + 100# convert to Goal_Position units
	for ID in rotation_id:
		command_srv('', ID, 'Goal_Position', target)

	# translation
	if (t / 10) % 2 == 0:
		target = target1
	else:
		target = target2
	command_srv('', translation_id, 'Moving_Speed', target)

	t += 1
	rate.sleep()

# stop the movement

if target == target1:
	command_srv('', translation_id, 'Moving_Speed', 1024)
else:
	command_srv('', translation_id, 'Moving_Speed', 0)
