#!/usr/bin/env python
import time
import rospy
class qwer_print(object):
    def __init__(self):
        for i in range (3):
            rospy.loginfo('****************TEST*********** %f' % i)

if __name__ == '__main__':
    rospy.init_node("qwer_print",anonymous=False)
    node_print = qwer_print()
    rospy.spin()
