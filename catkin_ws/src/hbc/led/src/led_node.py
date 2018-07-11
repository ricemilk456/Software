#!/usr/bin/env python
import rospy
from time import sleep
from duckietown_msgs.msg import Led

class LED():
    def __init__(self,debug=False):
        
        print "-----"
if __name__ == '__main__':
    rospy.init_node('led',anonymous=False)
    led = LED()
    rospy.spin()

