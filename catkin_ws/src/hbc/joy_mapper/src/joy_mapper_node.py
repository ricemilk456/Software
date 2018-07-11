#!/usr/bin/env python
import rospy
import numpy as np
import math
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped, Num
from sensor_msgs.msg import Joy
import time
from __builtin__ import True
from Adafruit_PWM_Servo_Driver import PWM
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class JoyMapper(object):

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))                
        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        self.led_state = False
        self.qrcode_state = False
        self.pwm = PWM(address=0x40,debug=False)

        # Setup Parameters
        self.v_gain = self.setupParam("~speed_gain", 0.8)
        self.omega_gain = self.setupParam("~steer_gain", 8.3)
        self.bicycle_kinematics = self.setupParam("~bicycle_kinematics", 0)
        self.steer_angle_gain = self.setupParam("~steer_angle_gain", 1)
        self.simulated_vehicle_length = self.setupParam("~simulated_vehicle_length", 0.18)

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_joy_override = rospy.Publisher("~joystick_override", BoolStamped, queue_size=1)
        self.pub_parallel_autonomy = rospy.Publisher("~parallel_autonomy",BoolStamped, queue_size=1)
        self.pub_anti_instagram = rospy.Publisher("anti_instagram_node/click",BoolStamped, queue_size=1)
        self.pub_e_stop = rospy.Publisher("wheels_driver_node/emergency_stop",BoolStamped,queue_size=1)
        self.pub_avoidance = rospy.Publisher("~start_avoidance",BoolStamped,queue_size=1)
        self.pub_led = rospy.Publisher("led",BoolStamped,queue_size=1)
        self.pub_switch = rospy.Publisher("~switchSpeed",BoolStamped,queue_size=1)
        self.pub_lcd_X = rospy.Publisher("lcd_X",BoolStamped,queue_size=1)
        self.pub_lcd_A = rospy.Publisher("lcd_A",BoolStamped,queue_size=1)
        self.pub_lcd_B = rospy.Publisher("lcd_B",BoolStamped,queue_size=1)
        self.pub_lcd_Y = rospy.Publisher("lcd_Y",BoolStamped,queue_size=1)
        self.pub_shoot = rospy.Publisher("shoot",BoolStamped,queue_size=1)
        self.pub_qrcode = rospy.Publisher("qrcode",BoolStamped,queue_size=1)
        self.pub_led = rospy.Publisher("led",Num,queue_size=1)
        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.sub_led = rospy.Subscriber("led", Num,self.cbLed, queue_size=1)
        
        # timer
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.publishControl)
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        self.has_complained = False

        self.state_parallel_autonomy = False
        self.state_verbose = False

        pub_msg = BoolStamped()
        pub_msg.data = self.state_parallel_autonomy
        pub_msg.header.stamp = self.last_pub_time
        self.pub_parallel_autonomy.publish(pub_msg)
    
    def cbLed(self, led_msg):
        led_msg = led_msg
        for i in range(10):
            print led_msg.num

    def decrease_speed (self):
        self.v_gain = rospy.get_param("~speed_gain")
        if self.v_gain > 0.21:
            rospy.set_param("~speed_gain",self.v_gain-0.1)
            print "[LB] decrease speed successfully, speed is {} now!".format(self.v_gain-0.1)
        else:
            print "[LB] speed gain is down to 0.2!"

    def increase_speed (self):
        self.v_gain = rospy.get_param("~speed_gain")
        if self.v_gain < 0.89:
             rospy.set_param("~speed_gain",self.v_gain+0.1)
             print "[RB] increase speed successfully, speed is {} now!".format(self.v_gain+0.1)
        else:
             print "[RB] speed gain is up to 0.9!"

    def cbParamTimer(self,event):
        self.v_gain = rospy.get_param("~speed_gain", 1.0)
        self.omega_gain = rospy.get_param("~steer_gain", 10)
    def lcd_off(self):        
            override_msg = BoolStamped()
            override_msg.data = False            
            self.pub_lcd_X.publish(override_msg)
            self.pub_lcd_A.publish(override_msg)
            self.pub_lcd_B.publish(override_msg)
            self.pub_lcd_Y.publish(override_msg)
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()
        self.processButtons(joy_msg)

    def publishControl(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = self.joy.header.stamp
        car_cmd_msg.v = self.joy.axes[1] * self.v_gain #Left stick V-axis. Up is positive
        if self.bicycle_kinematics:
            # Implements Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = self.joy.axes[2] * self.steer_angle_gain
            #steering_angle = self.joy.axes[0] * self.steer_angle_gain #using left stick can turn left or right
            car_cmd_msg.omega = car_cmd_msg.v / self.simulated_vehicle_length * math.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            car_cmd_msg.omega = self.joy.axes[2] * self.omega_gain
            #car_cmd_msg.omega = self.joy.axes[0] * self.omega_gain
        self.pub_car_cmd.publish(car_cmd_msg)

# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7,
# logitek = 8, left joy = 9, right joy = 10

    def processButtons(self, joy_msg):
        if (joy_msg.buttons[6] == 1): #The back button
            
            self.decrease_speed()
        elif (joy_msg.buttons[7] == 1): #the start button
            self.increase_speed()
        elif (joy_msg.buttons[5] == 1): # Right back button
            self.state_verbose ^= True
            rospy.loginfo('state_verbose = %s' % self.state_verbose)
            rospy.set_param('line_detector_node/verbose', self.state_verbose) # bad - should be published for all to hear - not set a specific param
        elif (joy_msg.buttons[4] == 1): #Left back button
            self.state_parallel_autonomy ^= True
            rospy.loginfo('state_parallel_autonomy = %s' % self.state_parallel_autonomy)
            parallel_autonomy_msg = BoolStamped()
            parallel_autonomy_msg.header.stamp = self.joy.header.stamp
            parallel_autonomy_msg.data = self.state_parallel_autonomy
            self.pub_parallel_autonomy.publish(parallel_autonomy_msg)
        elif (joy_msg.buttons[3] == 1): #Y button
            self.lcd_off() 
            self.qrcode_state  = not(self.qrcode_state)          
            lcd_state = BoolStamped()
            qr_state = BoolStamped()                         
            anti_instagram_msg = BoolStamped()
            anti_instagram_msg.header.stamp = self.joy.header.stamp
            anti_instagram_msg.data = True
            lcd_state.data = True
            qr_state.data = self.qrcode_state
            self.pub_anti_instagram.publish(anti_instagram_msg)
            self.pub_lcd_Y.publish(lcd_state)
            self.pub_qrcode.publish(qr_state)
    
        elif (joy_msg.buttons[8] == 1): #power button (middle)
            e_stop_msg = BoolStamped()
            e_stop_msg.header.stamp = self.joy.header.stamp
            e_stop_msg.data = True # note that this is toggle (actual value doesn't matter)
            self.pub_e_stop.publish(e_stop_msg)
        elif (joy_msg.buttons[9] == 1): #push left joystick button 
            avoidance_msg = BoolStamped()
            rospy.loginfo('start lane following with avoidance mode')
            avoidance_msg.header.stamp = self.joy.header.stamp
            avoidance_msg.data = True 
            self.pub_avoidance.publish(avoidance_msg)
        elif (joy_msg.buttons[0] == 1): #X button
            self.lcd_off() 
            led_msg = Num()
            led_msg.num = 100
            shoot_msg = BoolStamped()
            shoot_msg.data = True
            lcd_state = BoolStamped()
            lcd_state.data = True
            self.pub_led.publish(led_msg)
            self.pub_shoot.publish(shoot_msg)
            self.pub_lcd_X.publish(lcd_state)
        elif (joy_msg.buttons[1] == 1): # A button 
            self.lcd_off()
            override_msg = BoolStamped()
            lcd_state = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = False
            self.pub_joy_override.publish(override_msg)
            lcd_state.data = True
            self.pub_lcd_A.publish(lcd_state)
            print "[A] Auto mode!"
        elif (joy_msg.buttons[2] == 1): #B button
            self.lcd_off()
            lcd_state = BoolStamped()
            lcd_state.data = True
            override_msg = BoolStamped()
            override_msg.header.stamp = self.joy.header.stamp
            override_msg.data = True
            self.pub_joy_override.publish(override_msg)
            self.pub_lcd_B.publish(lcd_state)
            print "[B] Manual mode!"
        else:
            some_active = sum(joy_msg.buttons) > 0
            if some_active:
                rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))
                                          

if __name__ == "__main__":       
    rospy.init_node("joy_mapper",anonymous=False)
    joy_mapper = JoyMapper()
    rospy.spin()
