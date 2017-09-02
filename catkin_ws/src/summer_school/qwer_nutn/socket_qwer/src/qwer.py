#!/usr/bin/env python
import rospy
import socket
import pygame
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
from std_msgs.msg import String, Int32
from Adafruit_PWM_Servo_Driver import PWM
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
from __builtin__ import True

class Qwer_Player(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        self.pub_e_stop = rospy.Publisher("wheels_driver_node/emergency_stop",BoolStamped,queue_size=1)
        self.pub_joy_override = rospy.Publisher("/qwer/joy_mapper_node/joystick_override", BoolStamped, queue_size=1)
        self.pub_voice = rospy.Publisher("~Voice", String, queue_size=1)
        self.sub_voice = rospy.Subscriber("~Voice", String, self.cb_music, queue_size=1)
        self.sub_tag_id = rospy.Subscriber("/qwer/tag_detections_test", Int32, self.test, queue_size=1)
        self.flag = 0
        self.socket_PhoneCtrl()
    def test(self,Tag):
        tag_id = Int32()
        tag_id.data = Tag.data
        rospy.loginfo("Now found Id = %d-------------" %(tag_id.data))
        pygame.mixer.init()
        if tag_id.data == 350 and self.flag == 0:
            sound="/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/Desp.ogg"
            rospy.loginfo(' ---------Tag Music playing---------- ID = %d' %(tag_id.data))
            pygame.mixer.music.load(sound)
            pygame.mixer.music.play(0, 0)
            pygame.mixer.music.set_volume(0.05)  #The value argument is between 0.0 and 1.0
        elif tag_id.data == 351 and self.flag == 0:
            sound="/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/nier.ogg"
            rospy.loginfo(' ---------Tag Music playing---------- ID = %d' %(tag_id.data))
            pygame.mixer.music.load(sound)
            pygame.mixer.music.play(0, 0)
            pygame.mixer.music.set_volume(0.05)  #The value argument is between 0.0 and 1.0

        if tag_id.data == 350 or tag_id.data == 351:
            self.flag = 1
        else:
            self.flag = 0

    def cb_music(self,nowstr):
        pygame.mixer.init()
        if nowstr.data == '1':
            e_stop_msg = BoolStamped()
            e_stop_msg.data = True 
            self.pub_e_stop.publish(e_stop_msg)
            sound='/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/Desp.ogg'
            rospy.loginfo(' ---------Music playing----------  %s' %(nowstr.data))
            pygame.mixer.music.load(sound)
            pygame.mixer.music.play(0, 0)
            pygame.mixer.music.set_volume(0.05)  #The value argument is between 0.0 and 1.0
          #  while pygame.mixer.music.get_busy(): 
            #    pygame.time.Clock().tick(10)
        elif nowstr.data == '2':
            e_stop_msg = BoolStamped()
            e_stop_msg.data = True 
            self.pub_e_stop.publish(e_stop_msg)
            sound='/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/nier.ogg'
            rospy.loginfo(' ---------Music playing----------  %s' %(nowstr.data))
            pygame.mixer.music.load(sound)
            pygame.mixer.music.play(0, 0)
            pygame.mixer.music.set_volume(0.05)  #The value argument is between 0.0 and 1.0
           # while pygame.mixer.music.get_busy(): 
            #    pygame.time.Clock().tick(10)

    def socket_PhoneCtrl(self):
        #servo = PWM(0x40) #gripper used
        #servo.setPWMFreq(60) #gripper used
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.leftMotor = self.motorhat.getMotor(1)
        self.rightMotor = self.motorhat.getMotor(2)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rospy.loginfo("create socket succ!")
        sock.settimeout(120)    # if 20s it's no data received. it will interrupt
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)    #addr can reuse

        sock.bind(('', 50007))  #(HOST, PORT)
        rospy.loginfo("bind socket succ!")
        sock.listen(3)    #maximum connect 3 clients
        rospy.loginfo("listen success!")
        while True:
            rospy.loginfo("listen for client...")
            (conn, ADDR) = sock.accept()
            rospy.loginfo("get client")
            rospy.loginfo(ADDR)
            szBuf = conn.recv(1024)
            if szBuf == "1\n":
                rospy.loginfo("forward")
                self.leftMotor.setSpeed(120)
                self.rightMotor.setSpeed(120)
                self.leftMotor.run(Adafruit_MotorHAT.FORWARD)
                self.rightMotor.run(Adafruit_MotorHAT.FORWARD)
            elif szBuf == "2\n":
                rospy.loginfo("backward")
                self.leftMotor.setSpeed(120)
                self.rightMotor.setSpeed(120)
                self.leftMotor.run(Adafruit_MotorHAT.BACKWARD)
                self.rightMotor.run(Adafruit_MotorHAT.BACKWARD)
            elif szBuf == "3\n":
                rospy.loginfo("left")
                self.leftMotor.setSpeed(120)
                self.rightMotor.setSpeed(120)
                self.leftMotor.run(Adafruit_MotorHAT.BACKWARD)
                self.rightMotor.run(Adafruit_MotorHAT.FORWARD)
            elif szBuf =='4\n':
                rospy.loginfo("right")
                self.leftMotor.setSpeed(120)
                self.rightMotor.setSpeed(120)
                self.leftMotor.run(Adafruit_MotorHAT.FORWARD)
                self.rightMotor.run(Adafruit_MotorHAT.BACKWARD)
            elif szBuf == "5\n":
                rospy.loginfo('stop!!!')
                self.leftMotor.setSpeed(0)
                self.rightMotor.setSpeed(0)
                e_stop_msg = BoolStamped()
                e_stop_msg.data = True 
                self.pub_e_stop.publish(e_stop_msg)
            elif szBuf =='6\n':
                #self.state_verbose ^= True
                rospy.loginfo('state_verbose = %s') #% self.state_verbose)
                #rospy.set_param('line_detector_node/verbose', self.state_verbose)
                msg = String()
                msg.data = '1'
                self.pub_voice.publish(msg)
            elif szBuf =='7\n':
                override_msg = BoolStamped()
                override_msg.data = False
                self.pub_joy_override.publish(override_msg)
                sock.close()
                rospy.loginfo('socket has closing')
                break
            elif szBuf =='8\n':
                override_msg = BoolStamped()
                override_msg.data = True
                self.pub_joy_override.publish(override_msg)
                msg = String()
                msg.data = '2'
                self.pub_voice.publish(msg)


if __name__ == "__main__":
    rospy.init_node("qwer_player",anonymous=False)
    qwer_player = Qwer_Player()
    rospy.spin()

