#!/usr/bin/env python
import rospy
from time import sleep
from node_example.msg import ViewTemplate

class LED():
    def __init__(self,debug=False):
        sub = rospy.Subscriber('/topic_ex', ViewTemplate, self.cbText, queue_size=1)
        
    def cbText(self, msg):
        self.msg = msg
        print self.msg.custom_msg
        
if __name__ == '__main__':
    rospy.init_node('led',anonymous=False)
    led = LED()
    rospy.spin()