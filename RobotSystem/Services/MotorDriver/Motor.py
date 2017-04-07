from RobotSystem.Services.Utilities.RobotUtils import RobotUtils
import time
import math

class Motor(object):

	'''* servoMin == 125, servoMax == 525ms *'''
	'''* minVal>=0, maxVal<=100 *'''
	def __init__(self, horiz_value, pinNumber, minVal, maxVal, offset_to_center,name, pwm):

		if RobotUtils.MOTOR_DEBUG:
			print "Motor Init. name: ",name,"	| minValue: ",minVal,"	| maxValue: ",maxVal,"	| centerVal: ",centerVal

		self.pin_number = pinNumber

		self.servo_min = RobotUtils.SERVO_MIN
		self.servo_max = RobotUtils.SERVO_MAX

		self.min = minVal
		self.max = maxVal

		self.horiz_value = horiz_value
		self.center_value = horiz_value + offset_to_center
		self.value = self.center_value

		self.name = name

		self.pwm = pwm


		self.moveTo(self.center_value)

	def moveToHoriz(self):
		self.moveTo(self.horiz_value)

	def moveTo(self,val):
		if val > self.min and val < self.max:
			self.value = val

		elif val >= self.max:
			self.value = self.max

		elif val <= self.min:
			self.value = self.min

		else:
			print "something strange is happeneing..."

		scaled_value = int(RobotUtils.scale(self.value, RobotUtils.MIN_MOTOR_VALUE, RobotUtils.MAX_MOTOR_VALUE,  self.servo_min,self.servo_max))
		if scaled_value < self.servo_min:
			print "error: scaled_value < servo_min for: ",self.name

		elif scaled_value > self.servo_max:
			print "error: scaled_value < servo_min for: ",self.name

		else:
			if self.pwm != None:
				self.pwm.setPWM(self.pin_number, 0, scaled_value)

	def moveToInT(self,val,t):

		delta = self.value - val

		if val > self.min and val < self.max:
			self.value = val

		elif val >= self.max:
			self.value = self.max

		elif val <= self.min:
			self.value = self.min

		else:
			print "something strange is happeneing..."

		self.moveOffSetInT(delta,t)


	def moveOffSetInT(self,deltaVal,t):
		for i in range(int(math.fabs(deltaVal))):
			self.moveOffset( RobotUtils.PositiveOrNegative(deltaVal) )
			time.sleep(t)


	def moveOffset(self,deltaVal):
		val = self.value + deltaVal

		if val > self.min and val < self.max:
			self.value = val

		elif val >= self.max:
			self.value = self.max

		elif val <= self.min:
			self.value = self.min

		else:
			print "ERROR, very odd indeed"

		scaled_value = int(RobotUtils.scale(self.value, RobotUtils.MIN_MOTOR_VALUE, RobotUtils.MAX_MOTOR_VALUE, self.servo_min,self.servo_max))

		if self.pwm != None:
			self.pwm.setPWM(self.pin_number, 0, scaled_value)



	def reset(self):
		self.moveTo(self.center_value)
