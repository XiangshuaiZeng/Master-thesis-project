#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import Transform

rospy.init_node('end_effector_pos')
# create a tf2_ros type buffer
tfBuffer = tf2_ros.Buffer()
# create a TF2 transform listener object. Save data into tfBuffer
listener = tf2_ros.TransformListener(tfBuffer)

rate = rospy.Rate(200) # rate at 100 hz

# create a publisher to publish the info of end_effector
pub = rospy.Publisher('/end_effector_pos', Transform, queue_size=1)

while not rospy.is_shutdown():
    try:
        # Get last ( Time(0) )transform from the base frame to frame l7
        frame_info = tfBuffer.lookup_transform('world', 'laser_link', rospy.Time(0))
        end_pos = Transform()
        end_pos = frame_info.transform
        pub.publish(end_pos)
    except (tf2_ros.TransformException):
        rate.sleep()
        continue

    rate.sleep()
