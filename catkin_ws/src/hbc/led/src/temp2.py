from time import sleep
from Adafruit_PWM_Servo_Driver import PWM

class LED:
	def __init__(self,debug=False):
		self.pwm = PWM(address=0x40,debug=debug)
		for i in range(40):
			self.pwm.setPWM(i,0,4095)
		self.pwm.setPWM(0,4095,4095)
		sleep(100)
		self.pwm.setPWM(0,4095,4095)
		print "-----"
if __name__ == '__main__':
	led = LED()
