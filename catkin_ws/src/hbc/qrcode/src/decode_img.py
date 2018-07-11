#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
import time

def cbCImg(msg):   
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
    cv2.imwrite("/home/ubuntu/duckietown/catkin_ws/src/hbc/qrcode/src/qrcode_temp.jpeg",image_np)   
    
if __name__ == '__main__': 
    rospy.init_node('cv_test',anonymous=False)    
    sub_img = rospy.Subscriber("/hbc/camera_node/image/compressed",CompressedImage,cbCImg)
    rospy.spin()

