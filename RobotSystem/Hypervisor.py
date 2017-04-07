#!/usr/bin/python
from Services import *
import time
import math
import json
import sys
import threading
import os
from Queue import Queue,Empty

class Hypervisor():

	def __init__(self):

		if RobotUtils.LIVE_TESTING:
			self.pwm = PWM()
			self.pwm.setPWMFreq(RobotUtils.FREQUENCY)
		else:
			self.pwm = None


		self.agendaThreadAlive = True
		self.inputQueue = Queue()
		self.agendaThread = threading.Thread(group=None,target=self.updateAgendaLoop,name="agendaThread")
		self.agendaThread.start()

		self.data_file_name = RobotUtils.DATA_FILE

		self.front_left = None
		self.front_right = None
		self.back_left = None
		self.back_right = None

		self.TURN_LEFT = RobotUtils.TURN_LEFT
		self.TURN_RIGHT = RobotUtils.TURN_RIGHT
		self.FORWARD = RobotUtils.FORWARD
		self.BACKWARD = RobotUtils.BACKWARD
		self.STOP = RobotUtils.STOP
		self.AUTONOMOUS = RobotUtils.AUTONOMOUS
		self.INVALID_DATA_ERROR = RobotUtils.INVALID_DATA_ERROR

		self.horizVidMotor = Motor(50, RobotUtils.HORIZONTAL_VID_PIN, RobotUtils.HORIZONTAL_VID_MIN_VAL, RobotUtils.HORIZONTAL_VID_MAX_VAL, 0, "horizontal video motor", self.pwm)
		self.vertVidMotor = Motor( 50, RobotUtils.VERTICAL_VID_PIN, RobotUtils.VERTICAL_VID_MIN_VAL, RobotUtils.VERTICAL_VID_MAX_VAL, 0, "vertical video motor", self.pwm)

		self.setup()

		self.motors = [self.front_left, self.front_right,self.back_left,self.back_right, self.horizVidMotor, self.vertVidMotor ]

		self.MotionController = MotionController(self.TURN_LEFT,  self.TURN_RIGHT, self.FORWARD, self.BACKWARD, self.STOP,self.AUTONOMOUS,self.INVALID_DATA_ERROR,
		self.motors, RobotUtils
												)
		self.stand()
		RobotUtils.ColorPrinter(self.__class__.__name__, '__init__() finished. Robot Created with id ' +str(id(self)), 'OKBLUE')

	# loads json data and creates Leg objects with add_leg()
	def setup(self):

		with open(self.data_file_name) as data_file:
			data = json.load(data_file)
			constants = data["constants"]
			for i in range(len(data["legs"])):
				self.add_leg(data["legs"][i],constants)

	# reads dictuanary values from input, creates a Leg object, and adds it to leg variables
	def add_leg(self,legData,constants):

		leg_name = legData["name"]

		body_pin 				= legData["motors"]["body"]["pinValue"]
		body_offset 			= legData["motors"]["body"]["offset"]
		body_center 			= constants["bodyCenterValue"] + body_offset
		body_min 				= constants["bodyRange"]["min"]
		body_max 				= constants["bodyRange"]["max"]

		mid_horiz_value 		= legData["motors"]["middle"]["horizValue"]
		middle_pin 				= legData["motors"]["middle"]["pinValue"]
		middle_min 				= constants["middleRange"]["min"]
		middle_max 				= constants["middleRange"]["max"]
		middle_offset_to_center = constants["midOffsetFromHoriz"]

		leg_horiz_value 		= legData["motors"]["leg"]["horizValue"]
		leg_pin 				= legData["motors"]["leg"]["pinValue"]
		leg_min 				= constants["legRange"]["min"]
		leg_max 				= constants["legRange"]["max"]
		leg_offset_to_center 	= constants["legOffsetFromHoriz"]

		leg = Leg( self.pwm, leg_name, body_pin,	body_min,	body_max,	body_center, mid_horiz_value, 	middle_pin,	middle_min,	middle_max,	middle_offset_to_center, leg_horiz_value, leg_pin, leg_min,	leg_max, leg_offset_to_center)

		if leg_name == "FR":
			self.front_right = leg

		elif leg_name == "FL":
			self.front_left = leg

		elif leg_name == "BL":
			self.back_left = leg

		elif leg_name == "BR":
			self.back_right = leg

		else:
			print "ERROR: LEG CANNOT BE IDENTIFIED"

	# Called by server when a change in user data is detected
	def inputData(self,data):
		self.inputQueue.put(data)

	# Ends agenda loop thread
	def EndHypervisor(self):
		RobotUtils.ColorPrinter(self.__class__.__name__,'Ending Agenda Thread', 'FAIL')
		self.agendaThreadAlive = False

	def updateAgendaLoop(self):
		while True:
			if not self.agendaThreadAlive:
				self.agendaThread._Thread_stop()
			try:
				data = self.inputQueue.get_nowait()
				self.updateAgenda(data)
			except Empty:
				pass

			time.sleep(RobotUtils.AGENDA_UPDATE_SPEED)
		print '\033[94m' + "Robot: QUEUE READING FINISHED" + '\033[0m'
		sys.exit()

	# acts as central coordinator for the robot - raeads incoming data + state of the bot and calls methods accordingly
	def updateAgenda(self,data):
		self.MotionController.updateCameras(data)
		nextMove = self.MotionController.NextMove(data)
		if nextMove == self.INVALID_DATA_ERROR:
			print "Fix this"
		else:
			self.MotionController.MakeMove(nextMove)

	# Refer to stand()
	def reset(self):
		self.stand()

	# resets legs to default position
	def stand(self):
		self.front_left.reset()
		self.front_right.reset()
		self.back_left.reset()
		self.back_right.reset()

	def setAllHoriz(self):
		self.front_right.setMidAndLegHoriz()
		self.front_left.setMidAndLegHoriz()
		self.back_right.setMidAndLegHoriz()
		self.back_left.setMidAndLegHoriz()
		time.sleep(5)

	def setMidsToMin(self):
		self.front_right.middle.moveTo(self.front_right.middle.min)
		self.front_left.middle.moveTo(self.front_left.middle.min)
		self.back_left.middle.moveTo(self.back_left.middle.min)
		self.back_right.middle.moveTo(self.back_right.middle.min)
		time.sleep(10)

	def setMidsToMax(self):
		self.front_right.middle.moveTo(self.front_right.middle.max)
		self.front_left.middle.moveTo(self.front_left.middle.max)
		self.back_left.middle.moveTo(self.back_left.middle.max)
		self.back_right.middle.moveTo(self.back_right.middle.max)

	def turn(self,direction):
		print "method unimplemented"




	def lunge(self, FRB, FRM, FRL, FLB, FLM, FLL, BLB, BLM, BLL, BRB, BRM, BRL):
		splitNum = 10
		for x in range(splitNum):
			self.front_right.body.moveOffset(FRB/splitNum)
			self.front_right.middle.moveOffset(FRM/splitNum)
			self.front_right.leg.moveOffset(FRL/splitNum)

			self.front_left.body.moveOffset(FLB/splitNum)
			self.front_left.middle.moveOffset(FLM/splitNum)
			self.front_left.leg.moveOffset(FLL/splitNum)

			self.back_left.body.moveOffset(BLB/splitNum)
			self.back_left.middle.moveOffset(BLM/splitNum)
			self.back_left.leg.moveOffset(BLL/splitNum)

			self.back_right.body.moveOffset(BRB/splitNum)
			self.back_right.middle.moveOffset(BRM/splitNum)
			self.back_right.leg.moveOffset(BRL/splitNum)


	def forward(self):
		velocity = .01
		time_delay = .025
		std_piv_step_body_delta = -20
		std_piv_step_middle_delta = 50
		std_piv_step_leg_delta = 5

		self.front_left.standardPivotStep(std_piv_step_body_delta, std_piv_step_middle_delta, std_piv_step_leg_delta,velocity,time_delay*.01)
		time.sleep(time_delay)

		self.back_right.standardPivotStep(-std_piv_step_body_delta, std_piv_step_middle_delta, std_piv_step_leg_delta,velocity,time_delay)
		time.sleep(time_delay)

		self.back_right.standardPivotStep(-std_piv_step_body_delta, std_piv_step_middle_delta, std_piv_step_leg_delta,velocity,time_delay)
		time.sleep(time_delay)

		leg_extend_body_delta 	= 35
		leg_extend_middle_delta = -5
		leg_extend_leg_delta 	= 28

		self.front_right.legExtend( leg_extend_body_delta, leg_extend_middle_delta, leg_extend_leg_delta, velocity, time_delay)
		time.sleep(time_delay)

		splitNum = 10
		leg_condense_FLbody_delta = 40/splitNum
		leg_condense_BRbody_delta = -20/splitNum
		leg_condense_FRmiddle_delta = 20/splitNum
		leg_condense_FRleg_delta = -28/splitNum
		leg_condense_BLbody_delta = 20/splitNum
		leg_condense_BLmiddle_delta = -10/splitNum
		leg_condense_BLleg_delta = 28/splitNum


		# condense forward right
		for x in range(0, splitNum):
			self.front_left.body.moveOffset(leg_condense_FLbody_delta)
			self.back_right.body.moveOffset(leg_condense_BRbody_delta)
			self.front_right.middle.moveOffset(leg_condense_FRmiddle_delta)
			self.front_right.leg.moveOffset(leg_condense_FRleg_delta)
			self.back_left.body.moveOffset(leg_condense_BLbody_delta)
			self.back_left.middle.moveOffset(leg_condense_BLmiddle_delta)
			self.back_left.leg.moveOffset(leg_condense_BLleg_delta)

		leg_step_BLbody_delta = -30
		leg_step_BLmiddle_delta = 30
		leg_step_BLleg_delta = -28
		time.sleep(time_delay)

		# back left standard pivot step with mid offset"
		self.back_left.standardPivotStepWithMidMovement(leg_step_BLbody_delta, leg_step_BLmiddle_delta, leg_step_BLleg_delta,velocity,time_delay)

		leg_step_FRbody_delta = -40
		leg_step_FRmiddle_delta = 5
		leg_step_FRleg_delta = 28

		# front left standard pivot step with mid movement"
		self.front_left.standardPivotStepWithMidMovement(leg_step_FRbody_delta, leg_step_FRmiddle_delta, leg_step_FRleg_delta, velocity,time_delay)
		time.sleep(time_delay)

		frontRightBodySplitDiff = self.front_right.body.center_value - self.front_right.body.value
		frontRightMiddleSplitDiff =self.front_right.middle.value - self.front_right.middle.center_value
		frontRightLegSplitDiff = self.front_right.leg.value - self.front_right.leg.center_value

		frontLeftBodySplitDiff = self.front_left.body.center_value - self.front_left.body.value
		frontLeftMiddleSplitDiff =self.front_left.middle.center_value  - self.front_left.middle.value
		frontLeftLegSplitDiff = self.front_left.leg.center_value - self.front_left.leg.value

		backRightBodySwing = -20/splitNum
		backRightMiddleSwing = -10/splitNum
		backRightLegSwing = 28/splitNum
		backLeftBodySwing = 40/splitNum

		# forward condence
		for x in range(0, splitNum):
			self.front_right.body.moveOffset(frontRightBodySplitDiff/splitNum)
			self.front_right.middle.moveOffset(frontRightMiddleSplitDiff/splitNum)
			self.front_right.leg.moveOffset(frontRightLegSplitDiff/splitNum)

			#self.front_left.body.moveOffset(frontLeftBodySplitDiff/splitNum)
			self.front_left.middle.moveOffset(frontLeftMiddleSplitDiff/splitNum)
			self.front_left.leg.moveOffset(frontLeftLegSplitDiff/splitNum)

			self.back_right.body.moveOffset(backRightBodySwing)
			self.back_right.middle.moveOffset(backRightMiddleSwing)
			self.back_right.leg.moveOffset(backRightLegSwing)
			self.back_left.body.moveOffset(backLeftBodySwing)

		time.sleep(time_delay)

		leg_step_BRbody_delta = 30
		leg_step_BRmiddle_delta = 30
		leg_step_BRleg_delta = -28
		time.sleep(time_delay)

		self.back_right.standardPivotStepWithMidMovement(leg_step_BRbody_delta, leg_step_BRmiddle_delta, leg_step_BRleg_delta,velocity,time_delay)

		leg_extend_body_delta = 35
		leg_extend_middle_delta =-5
		leg_extend_leg_delta = 28

		self.front_right.legExtend( leg_extend_body_delta, leg_extend_middle_delta, leg_extend_leg_delta, velocity, time_delay)
		time.sleep(time_delay)

		RlungeFLbody= 40
		RlungeBRbody= -20
		RlungeFRmiddle = 30
		RlungeFRleg = -28
		RlungeBLmiddle = -10
		RlungeBLleg = 28

		self.lunge(0,RlungeFRmiddle,RlungeFRleg,RlungeFLbody,0,0, 0,RlungeBLmiddle,RlungeBLleg,RlungeBRbody,0,0)

		leg_step_BLbody_delta = -30
		leg_step_BLmiddle_delta = 30
		leg_step_BLleg_delta = -28
		time.sleep(time_delay)

		self.back_left.standardPivotStepWithMidMovement(leg_step_BLbody_delta, leg_step_BLmiddle_delta, leg_step_BLleg_delta,velocity,time_delay)

		self.front_left.legExtend( -leg_extend_body_delta, leg_extend_middle_delta, leg_extend_leg_delta, velocity, time_delay)
		time.sleep(time_delay)

		LlungeFRbody= -40
		LlungeBLbody= 20
		LlungeFLmiddle = 30
		LlungeFLleg = -28
		LlungeBRmiddle = -10
		LlungeBRleg = 28

		self.lunge(LlungeFRbody, 0,0,0,LlungeFLmiddle,LlungeFLleg, LlungeBLbody,0,0 ,0,LlungeBRmiddle, LlungeBRleg)
		self.reset()
		time.sleep(10)
		self.forward()


	# method to develop walking motion
	def walkInit(self):

		velocity = .01
		time_delay = .025

		if self.forwardInc < 8:


			velocity = .01
			time_delay = .025

			std_piv_step_body_delta = -20
			std_piv_step_middle_delta = 50
			std_piv_step_leg_delta = 5

			if self.forwardInc == 1:
				self.front_left.standardPivotStep(std_piv_step_body_delta, std_piv_step_middle_delta, std_piv_step_leg_delta,velocity,time_delay*.01)
				time.sleep(time_delay)
				#self.updateAgenda()

			elif self.forwardInc == 2:
				self.back_right.standardPivotStep(-std_piv_step_body_delta, std_piv_step_middle_delta, std_piv_step_leg_delta,velocity,time_delay)
				time.sleep(time_delay)
				#self.updateAgenda()

			else:
				print "self.forwardInc == else"


			'''
			if self.forwardInc == 1:
				print " walkInit with self.forwardInc:	",self.forwardInc
				#print "front left pivot step"


			elif self.forwardInc == 2:
				print " walkInit with self.forwardInc:	",self.forwardInc
				#print "back right pivot step"
				self.back_right.standardPivotStep(-std_piv_step_body_delta, std_piv_step_middle_delta, std_piv_step_leg_delta,velocity,time_delay)
				time.sleep(time_delay)
				self.updateAgenda()

			leg_extend_body_delta 	= 35
			leg_extend_middle_delta = -5
			leg_extend_leg_delta 	= 28


			elif self.forwardInc == 3:
				print " walkInit with self.forwardInc:	",self.forwardInc
				print "front right leg extend"
				#self.front_right.legExtend( leg_extend_body_delta, leg_extend_middle_delta, leg_extend_leg_delta, velocity, time_delay)
				time.sleep(time_delay)
				self.updateAgenda()


			splitNum = 10
			leg_condense_FLbody_delta = 40/splitNum
			leg_condense_BRbody_delta = -20/splitNum
			leg_condense_FRmiddle_delta = 20/splitNum
			leg_condense_FRleg_delta = -28/splitNum
			leg_condense_BLbody_delta = 20/splitNum
			leg_condense_BLmiddle_delta = -10/splitNum
			leg_condense_BLleg_delta = 28/splitNum


			elif (self.forwardInc == 4):
				print " walkInit with self.forwardInc:	",self.forwardInc
				#print "condense forward right"
				for x in range(0, splitNum):
					self.front_left.body.moveOffset(leg_condense_FLbody_delta)
					self.back_right.body.moveOffset(leg_condense_BRbody_delta)
					self.front_right.middle.moveOffset(leg_condense_FRmiddle_delta)
					self.front_right.leg.moveOffset(leg_condense_FRleg_delta)
					self.back_left.body.moveOffset(leg_condense_BLbody_delta)
					self.back_left.middle.moveOffset(leg_condense_BLmiddle_delta)
					self.back_left.leg.moveOffset(leg_condense_BLleg_delta)
				self.updateAgenda()


			leg_step_BLbody_delta = -30
			leg_step_BLmiddle_delta = 30
			leg_step_BLleg_delta = -28
			time.sleep(time_delay)


			elif self.forwardInc == 5:
				print " walkInit with self.forwardInc:	",self.forwardInc
				#print "back left standard pivot step with mid offset"
				self.back_left.standardPivotStepWithMidMovement(leg_step_BLbody_delta, leg_step_BLmiddle_delta, leg_step_BLleg_delta,velocity,time_delay)
				self.updateAgenda()

			leg_step_FRbody_delta = -40
			leg_step_FRmiddle_delta = 5
			leg_step_FRleg_delta = 28

			elif self.forwardInc == 6:
				print " walkInit with self.forwardInc:	",self.forwardInc
				#print "front left standard pivot step with mid movement"
				self.front_left.standardPivotStepWithMidMovement(leg_step_FRbody_delta, leg_step_FRmiddle_delta, leg_step_FRleg_delta, velocity,time_delay)
				time.sleep(time_delay)
				self.updateAgenda()

			frontRightBodySplitDiff = self.front_right.body.center_value - self.front_right.body.value
			frontRightMiddleSplitDiff =self.front_right.middle.value - self.front_right.middle.center_value
			frontRightLegSplitDiff = self.front_right.leg.value - self.front_right.leg.center_value

			frontLeftBodySplitDiff = self.front_left.body.center_value - self.front_left.body.value
			frontLeftMiddleSplitDiff =self.front_left.middle.center_value  - self.front_left.middle.value
			frontLeftLegSplitDiff = self.front_left.leg.center_value - self.front_left.leg.value

			backRightBodySwing = -20/splitNum
			backRightMiddleSwing = -10/splitNum
			backRightLegSwing = 28/splitNum
			backLeftBodySwing = 40/splitNum


			elif self.forwardInc == 7:
				print " walkInit with self.forwardInc:	",self.forwardInc
				#print "forward condence"
				for x in range(0, splitNum):
					self.front_right.body.moveOffset(frontRightBodySplitDiff/splitNum)
					self.front_right.middle.moveOffset(frontRightMiddleSplitDiff/splitNum)
					self.front_right.leg.moveOffset(frontRightLegSplitDiff/splitNum)
					#spacer
					#self.front_left.body.moveOffset(frontLeftBodySplitDiff/splitNum)
					self.front_left.middle.moveOffset(frontLeftMiddleSplitDiff/splitNum)
					self.front_left.leg.moveOffset(frontLeftLegSplitDiff/splitNum)

					self.back_right.body.moveOffset(backRightBodySwing)
					self.back_right.middle.moveOffset(backRightMiddleSwing)
					self.back_right.leg.moveOffset(backRightLegSwing)
					self.back_left.body.moveOffset(backLeftBodySwing)

				time.sleep(time_delay)
				self.updateAgenda()

			else:
				print "This message should not appear"
			'''

		else:
			self.walkCont(time_delay,velocity,1)



	def walkCont(self,time_delay,velocity,timesThrough):

		# THIS WILL NOT WORK. KNOWN BUG
		if(timesThrough ==2):
			print "times through == 2, reseting to avoid offset buildup"
			self.reset()
			self.forwardInc == self.forwardIncMin
			self.walkInit()

		leg_step_BRbody_delta = 30
		leg_step_BRmiddle_delta = 30
		leg_step_BRleg_delta = -28
		time.sleep(time_delay)
		'''
		if self.forwardInc  == 8:
			print " walkCont with self.forwardInc:	",self.forwardInc
			#print "back right standard pivot step with mid offset"
			self.back_right.standardPivotStepWithMidMovement(leg_step_BRbody_delta, leg_step_BRmiddle_delta, leg_step_BRleg_delta,velocity,time_delay)
			self.updateAgenda()

		elif self.forwardInc == 9:

			leg_extend_body_delta = 35
			leg_extend_middle_delta =-5
			leg_extend_leg_delta = 28

			print " walkCont with self.forwardInc:	",self.forwardInc
			#print "front right leg extend"
			self.front_right.legExtend( leg_extend_body_delta, leg_extend_middle_delta, leg_extend_leg_delta, velocity, time_delay)
			time.sleep(time_delay)
			self.updateAgenda()

		RlungeFLbody= 40
		RlungeBRbody= -20
		RlungeFRmiddle = 30
		RlungeFRleg = -28
		RlungeBLmiddle = -10
		RlungeBLleg = 28

		elif self.forwardInc == 10:
			self.lunge(0,RlungeFRmiddle,RlungeFRleg,RlungeFLbody,0,0, 0,RlungeBLmiddle,RlungeBLleg,RlungeBRbody,0,0)
			self.updateAgenda()

		leg_step_BLbody_delta = -30
		leg_step_BLmiddle_delta = 30
		leg_step_BLleg_delta = -28
		time.sleep(time_delay)

		elif self.forwardInc == 11:
			print " walkCont with self.forwardInc:	",self.forwardInc
			#print "back left standard pivot step with mid offset"
			self.back_left.standardPivotStepWithMidMovement(leg_step_BLbody_delta, leg_step_BLmiddle_delta, leg_step_BLleg_delta,velocity,time_delay)
			self.updateAgenda()

		elif self.forwardInc == 12:
			print " walkCont with self.forwardInc:	",self.forwardInc
			#print "front left leg extend"
			self.front_left.legExtend( -leg_extend_body_delta, leg_extend_middle_delta, leg_extend_leg_delta, velocity, time_delay)
			time.sleep(time_delay)
			self.updateAgenda()

		LlungeFRbody= -40
		LlungeBLbody= 20
		LlungeFLmiddle = 30
		LlungeFLleg = -28
		LlungeBRmiddle = -10
		LlungeBRleg = 28

		elif self.forwardInc == 13:
			print " walkCont with self.forwardInc:	",self.forwardInc
			self.lunge(LlungeFRbody, 0,0,0,LlungeFLmiddle,LlungeFLleg, LlungeBLbody,0,0 ,0,LlungeBRmiddle, LlungeBRleg)
			self.forwardInc == 7
			self.updateAgenda()

		else:
			print "this print statement should never be visible. error."
		'''
