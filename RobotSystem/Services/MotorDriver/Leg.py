from Motor import Motor
import time,math,json

class Leg(object):

	def __init__(self, pwm,name, body_pin,	body_min,	body_max,	body_center, mid_horiz_value, 	middle_pin,	middle_min,	middle_max,	middle_offset_to_center, leg_horiz_value, 	leg_pin,	leg_min,	leg_max,	leg_offset_to_center):


		self.pwm = pwm

		self.name = name

		self.body_name = "body"+name
		self.middle_name = "middle"+name
		self.leg_name = "leg"+name

		self.body 	= Motor( 0,					body_pin,	body_min,	body_max,	body_center,				self.body_name,		pwm)
		self.middle = Motor( mid_horiz_value, 	middle_pin,	middle_min,	middle_max,	middle_offset_to_center,	self.middle_name,	pwm)
		self.leg 	= Motor( leg_horiz_value, 	leg_pin,	leg_min,	leg_max,	leg_offset_to_center,		self.leg_name,		pwm)

	# velocity		- speed of movement (second delay between incrememnts in position)
	# Standard step in which the middle and leg lift up, the body rotates, and the middle + leg return to starting positions
	def standardPivotStep(self,body_delta,middle_delta,leg_delta,velocity,time_delay):

		middleEndVal = self.middle.value
		legEndVal = self.leg.value

		self.middle.moveOffSetInT(middle_delta, velocity)
		time.sleep(time_delay)

		self.leg.moveOffSetInT(leg_delta,velocity)
		time.sleep(time_delay)

		self.body.moveOffSetInT(body_delta,velocity)
		time.sleep(time_delay)

		self.middle.moveTo(middleEndVal)
		time.sleep(time_delay)

		self.leg.moveTo(legEndVal)


    # std_pvt_body_delta,	std_pvt_middle_delta,	std_pvt_leg_delta,	velocity/10,	time_delay/10)
		
	# velocity		- speed of movement (second delay between incrememnts in position)
	# Specific motion in which the robot lifts the middle and leg, pivots the body and drops the leg in front of the robot
	def legExtend(self, body_delta, middle_delta, leg_delta, velocity, time_delay):
		middle_raise= 30

		self.middle.moveOffset(middle_raise)
		self.leg.moveOffSetInT(leg_delta,velocity)
		time.sleep(time_delay)

		self.body.moveOffSetInT(body_delta,velocity)
		time.sleep(time_delay)
		self.middle.moveOffSetInT(middle_delta - middle_raise, velocity)
		time.sleep(time_delay)


	def setMidAndLegHoriz(self):
		self.middle.moveToHoriz()
		self.leg.moveToHoriz()


	def reset(self):
		self.body.reset()
		self.middle.reset()
		self.leg.reset()

	# setLegToXY moves the tip of the leg to an xy point
	# x	-  horozontal distance to middle servo nut, must be POSITVE
	# y	-  vertical distance from the ground to the middle servo   **NEED TO CHECK IF CAN BE NEG**
	def setLegXY(self, x, y):

		print "this method has not been completed"

	def standardPivotStepWithMidMovement(self,body_delta,middle_delta,leg_delta,velocity,time_delay):

		middlePickUp = 40

		self.middle.moveOffSetInT(middlePickUp, velocity)
		time.sleep(time_delay)

		self.body.moveOffSetInT(body_delta,velocity)
		time.sleep(time_delay)

		self.leg.moveOffSetInT(leg_delta,velocity)
		time.sleep(time_delay)

		self.middle.moveOffset(middle_delta - middlePickUp)
