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
        self.sub_voice = rospy.Subscriber("~Voice", String, self.cb_SoundPlayer, queue_size=1)
        self.sub_tag_id = rospy.Subscriber("/qwer/tag_detections_test", Int32, self.get_Apriltag, queue_size=1)

        #set global variable
        self.flag = 0
        self.sound = ''
        self.n_stop = False

        #Autostart lane_following
        override_msg = BoolStamped()
        override_msg.data = False
        self.pub_joy_override.publish(override_msg)

        #Start socket function
        self.socket_Ctrl()

    def get_Apriltag(self,Tag):
        tag_id = Int32()
        tag_id.data = Tag.data
        rospy.loginfo("Now found Id = %d-------------   and flag = %d" %(tag_id.data, self.flag))
        if tag_id.data == 350 and self.flag == 0:
            self.sound="/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/DesGlaneuses.ogg"
            rospy.loginfo(' ---------Found Tag playing guide vocie-----------')
            e_stop_msg = BoolStamped()
            e_stop_msg.data = True 
            self.pub_e_stop.publish(e_stop_msg)
            msg = String()
            msg.data = 'at def get_apriltag(self,Tag) and tag_id.data = 350'
            self.pub_voice.publish(msg)
        elif tag_id.data == 351 and self.flag == 0:
            self.sound="/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/LAngelus.ogg"
            rospy.loginfo(' ---------Found Tag playing guide vocie-----------')
            e_stop_msg = BoolStamped()
            e_stop_msg.data = True 
            self.pub_e_stop.publish(e_stop_msg)
            msg = String()
            msg.data = 'at def get_apriltag(self,Tag) and tag_id.data = 351'
            self.pub_voice.publish(msg)
        elif tag_id.data == 352 and self.flag == 0:
            self.sound="/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/Shepherdess.ogg"
            rospy.loginfo(' ---------Found Tag playing guide vocie-----------')
            e_stop_msg = BoolStamped()
            e_stop_msg.data = True 
            self.pub_e_stop.publish(e_stop_msg)
            msg = String()
            msg.data = 'at def get_apriltag(self,Tag) and tag_id.data = 352'
            self.pub_voice.publish(msg)
        elif tag_id.data == 353 and self.flag == 0:
            self.sound="/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/Desp.ogg"
            rospy.loginfo(' ---------Found Tag playing guide vocie-----------')
            e_stop_msg = BoolStamped()
            e_stop_msg.data = True 
            self.pub_e_stop.publish(e_stop_msg)
            msg = String()
            msg.data = 'at def get_apriltag(self,Tag) and tag_id.data = 353'
            self.pub_voice.publish(msg)


        if tag_id.data == 0:
            self.flag = 0
        else:
            self.flag = 1

    def cb_SoundPlayer(self,nowstr):
        pygame.mixer.init()
        rospy.loginfo("Welcome to SoundPlayer ^_^")
        if nowstr.data == 'pause mode':
            self.n_stop = not self.n_stop
            if self.n_stop == True:
                pygame.mixer.music.pause()
                rospy.loginfo("[%s]: The Voice stop Activated" % (nowstr.data))
            else:
                pygame.mixer.music.unpause()
                rospy.loginfo("[%s]: The Voice stop Released" % (nowstr.data))
        else:
            rospy.loginfo("[%s] ------- sound's location=%s" %(nowstr.data, self.sound))
            self.n_stop = False #reset the start/pause State
            pygame.mixer.music.load(self.sound)
            pygame.mixer.music.play(0, 0)
            pygame.mixer.music.set_volume(1)  #The value argument is between 0.0 and 1.0
           # while pygame.mixer.music.get_busy():  #it will play sound until the sound finished
            #    pygame.time.Clock().tick(10)

    def socket_Ctrl(self):
        #servo = PWM(0x40) #gripper used
        #servo.setPWMFreq(60) #gripper used
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.leftMotor = self.motorhat.getMotor(1)
        self.rightMotor = self.motorhat.getMotor(2)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        rospy.loginfo("create socket succ!")
        sock.settimeout(1000)    # if 1000s it's no data received. it will interrupt
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)    #addr can reuse
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)    #keep connect

        sock.bind(('', 50007))  #(HOST, PORT)
        rospy.loginfo("bind socket succ!")
        sock.listen(2)    #maximum connect 3 clients
        rospy.loginfo("listen success!")
        while True:
            rospy.loginfo("listen for client...")
            (conn, ADDR) = sock.accept()
            rospy.loginfo("get client")
            rospy.loginfo(ADDR)
            szBuf = conn.recv(1024)
            rospy.loginfo("i recv ----------- szBuf= %s" % (szBuf))
            if szBuf == "1\n":
                rospy.loginfo("forward")
                #self.leftMotor.setSpeed(120)
                #self.rightMotor.setSpeed(120)
                #self.leftMotor.run(Adafruit_MotorHAT.FORWARD)
                #self.rightMotor.run(Adafruit_MotorHAT.FORWARD)
                self.sound="/home/ubuntu/duckietown/catkin_ws/src/summer_school/qwer_nutn/socket_qwer/include/socket_qwer/nier.ogg"
                msg = String()
                msg.data = 'using app button'
                self.pub_voice.publish(msg)
            elif szBuf == "2\n":
                rospy.loginfo("backward")
                #self.leftMotor.setSpeed(120)
                #self.rightMotor.setSpeed(120)
                #self.leftMotor.run(Adafruit_MotorHAT.BACKWARD)
                #self.rightMotor.run(Adafruit_MotorHAT.BACKWARD)
                sock.close()
                rospy.loginfo('socket has closing')
                break
            elif szBuf == "3\n":
                rospy.loginfo("left")
                #self.leftMotor.setSpeed(120)
                #self.rightMotor.setSpeed(120)
                #self.leftMotor.run(Adafruit_MotorHAT.BACKWARD)
                #self.rightMotor.run(Adafruit_MotorHAT.FORWARD)
                msg = String()
                msg.data = 'repeat now Giude voice'
                self.pub_voice.publish(msg)
            elif szBuf =='4\n':
                rospy.loginfo("right")
                #self.leftMotor.setSpeed(120)
                #self.rightMotor.setSpeed(120)
                #self.leftMotor.run(Adafruit_MotorHAT.FORWARD)
                #self.rightMotor.run(Adafruit_MotorHAT.BACKWARD)
                msg = String()
                msg.data = 'pause mode'
                self.pub_voice.publish(msg)
            elif szBuf == "5\n":
                rospy.loginfo('stop!!!')
                #self.leftMotor.setSpeed(0)
                #self.rightMotor.setSpeed(0)
                e_stop_msg = BoolStamped()
                e_stop_msg.data = True 
                self.pub_e_stop.publish(e_stop_msg)
            elif szBuf =='6\n':
                self.state_verbose ^= True
                rospy.loginfo('state_verbose = %s' % self.state_verbose)
                rospy.set_param('line_detector_node/verbose', self.state_verbose)
            elif szBuf =='7\n':
                override_msg = BoolStamped()
                override_msg.data = False
                self.pub_joy_override.publish(override_msg)
            elif szBuf =='8\n':
                override_msg = BoolStamped()
                override_msg.data = True
                self.pub_joy_override.publish(override_msg)
            elif szBuf == '1':
                rospy.loginfo("receive from linkit : %s" %(szBuf))
                e_stop_msg = BoolStamped()
                e_stop_msg.data = True 
                self.pub_e_stop.publish(e_stop_msg)
            elif szBuf == '2':
                rospy.loginfo("receive from linkit : %s" %(szBuf))
                msg = String()
                msg.data = 'repeat now Giude voice'
                self.pub_voice.publish(msg)
            elif szBuf == '3':
                rospy.loginfo("receive from linkit : %s" %(szBuf))
                msg = String()
                msg.data = 'pause mode'
                self.pub_voice.publish(msg)

if __name__ == "__main__":
    rospy.init_node("qwer_player",anonymous=False)
    qwer_player = Qwer_Player()
    rospy.spin()

