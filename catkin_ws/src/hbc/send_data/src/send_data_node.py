#!/usr/bin/env python
import sys
import os
import rospy
import time
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
import gspread
import urllib3
urllib3.disable_warnings()
from oauth2client.service_account import ServiceAccountCredentials as SAC

def shoot():
    command = "fswebcam -i 0 -d v4l2:/dev/video0 --no-banner -p YUYV --jpeg 95 --save /home/ubuntu/duckietown/catkin_ws/src/hbc/send_data/src/shoot.jpg"
    os.system(command)
    print "shoot successfully!!"

class Send_data(object):
    def __init__(self,debug=False):
        self.sub_shoot = rospy.Subscriber("shoot", BoolStamped, self.cbShoot, queue_size=1)
    def cbShoot(self, data_msg):
        shoot()

if __name__ == '__main__':
    rospy.init_node("send_data",anonymous=False)
    send_data = Send_data()
    rospy.spin()