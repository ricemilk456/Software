#! /usr/bin/env python

import rospy
from node_example.msg import ViewTemplate
count = 0

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/topic_ex', ViewTemplate, queue_size=1)


rate = rospy.Rate(2)
message = ViewTemplate()
count += 1
message.custom_msg = "Hello World    "
message.custom_msg += str(count)

while not rospy.is_shutdown():
    pub.publish(message)
    rate.sleep()