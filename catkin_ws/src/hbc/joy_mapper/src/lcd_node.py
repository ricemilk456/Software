#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import  Twist2DStamped, BoolStamped
import time
import smbus

I2C_ADDR  = 0x3f
LCD_WIDTH = 16

LCD_CHR = 1
LCD_CMD = 0

LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_LINE_3 = 0x94
LCD_LINE_4 = 0xD4

LCD_BACKLIGHT  = 0x08
#LCD_BACKLIGHT = 0x00

ENABLE = 0b00000100

E_PULSE = 0.0005
E_DELAY = 0.0005

#bus = smbus.SMBus(0)
bus = smbus.SMBus(1)

def lcd_init():
    lcd_byte(0x33, LCD_CMD) # 110011 Initialise
    lcd_byte(0x32, LCD_CMD) # 110010 Initialise
    lcd_byte(0x06, LCD_CMD) # 000110 Cursor move direction
    lcd_byte(0x0C, LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
    lcd_byte(0x28, LCD_CMD) # 101000 Data length, number of lines, font size
    lcd_byte(0x01, LCD_CMD) # 000001 Clear display
    time.sleep(E_DELAY)

def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)

    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    time.sleep(E_DELAY)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(E_PULSE)
    bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
    time.sleep(E_DELAY)

def lcd_string(message,line):
    message = message.ljust(LCD_WIDTH, " ")

    lcd_byte(line, LCD_CMD)

    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]),LCD_CHR)

def main(state):
    if state == '1':
        lcd_init()
        lcd_string("X mode", LCD_LINE_1)
        lcd_string("Capture Photo", LCD_LINE_2)
    elif state == '2':
        lcd_init()
        lcd_string("A mode", LCD_LINE_1)
        lcd_string("Auto mode", LCD_LINE_2)
    elif state == '3':
        lcd_init()
        lcd_string("B mode", LCD_LINE_1)
        lcd_string("Manual mode", LCD_LINE_2)
    elif state == '4':
        lcd_init()
        lcd_string("Y mode", LCD_LINE_1)
        lcd_string("Scanning QRcode", LCD_LINE_2)
       
class Lcd(object):
    def __init__(self):
        self.sub_lcd_X = rospy.Subscriber("lcd_X", BoolStamped,self.cbLcd_X, queue_size=1)
        self.sub_lcd_A = rospy.Subscriber("lcd_A", BoolStamped,self.cbLcd_A, queue_size=1)
        self.sub_lcd_B = rospy.Subscriber("lcd_B", BoolStamped,self.cbLcd_B, queue_size=1)
        self.sub_lcd_Y = rospy.Subscriber("lcd_Y", BoolStamped,self.cbLcd_Y, queue_size=1)
    def cbLcd_X(self, lcd_msg):               
        self.lcd = lcd_msg        
        if self.lcd.data == True:            
            main('1')
    def cbLcd_A(self, lcd_msg):  
        self.lcd = lcd_msg 
        if self.lcd.data == True:
            main('2')
    def cbLcd_B(self, lcd_msg):
        self.lcd = lcd_msg 
        if self.lcd.data == True:
            main('3')
    def cbLcd_Y(self, lcd_msg):
        self.lcd = lcd_msg   
        if self.lcd.data == True:
            main('4')
if __name__ == '__main__':
    rospy.init_node("lcd",anonymous=False)
    lcd = Lcd()
    rospy.spin()