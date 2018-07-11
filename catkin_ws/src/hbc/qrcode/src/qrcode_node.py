#!/usr/bin/env python
import rospy
import os
import time
import sys
from duckietown_msgs.msg import BoolStamped
from picamera import PiCamera
from time import sleep
import zbar
from PIL import Image
import gspread
import urllib3
urllib3.disable_warnings()
from oauth2client.service_account import ServiceAccountCredentials as SAC

def identify():
    scanner = zbar.ImageScanner()
    scanner.parse_config('enable')
    while True:
        try:
            img = Image.open("/home/ubuntu/duckietown/catkin_ws/src/hbc/qrcode/src/qrcode_temp.jpeg").convert('L')
            break
        except:
            pass
    width, height = img.size
    
    qrCode = zbar.Image(width, height, 'Y800', img.tobytes())
    scanner.scan(qrCode)
    data = ''
    for s in qrCode:
        data += s.data
    if data == "":
        print "empty\n"
    else:
        print data,"\n"
        rospy.set_param("~data",data)
    return data

def Send_Data():
    GDriveJSON = '/home/ubuntu/duckietown/catkin_ws/src/hbc/send_data/src/duckietown3.json'
    GSpreadSheet = 'duckietown'
    WaitSecond = 0.5
    try:
        scope = ['https://spreadsheets.google.com/feeds','https://www.googleapis.com/auth/drive']

        key = SAC.from_json_keyfile_name(GDriveJSON, scope)
        gc = gspread.authorize(key)
        worksheet = gc.open(GSpreadSheet).sheet1

    except Exception as ex:
        print('failed to connect Google excel', ex)
        sys.exit(1)
    Data = rospy.get_param("~data")
    sysTime = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) 
    worksheet.append_row((sysTime,Data))
    print "upload to GoogleSheet successfully!!"

    time.sleep(WaitSecond)

class Qrcode(object):
    
    def __init__(self,debug=False):
        self.pub_data= rospy.Publisher("send_data", BoolStamped, queue_size=1)
        self.sub_data = rospy.Subscriber("send_data", BoolStamped, self.cbSend_data, queue_size=1)
        self.sub_qrcode = rospy.Subscriber("qrcode", BoolStamped, self.qrcode, queue_size=1)
        
        
        
    def qrcode(self, data_msg):
        for i in range(10):       
            if identify() != "":
                override_msg = BoolStamped()                
                override_msg.data = True
                self.pub_data.publish(override_msg)
            
    def cbSend_data(self, data_msg):
        Send_Data()

if __name__ == '__main__':
    rospy.init_node("qrcode",anonymous=False)
    qrcode = Qrcode()
    rospy.spin()
