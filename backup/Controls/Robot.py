#!/usr/bin/python
from RobotUtil import RobotUtils
if RobotUtils.LIVE_TESTING:
	from pwm import PWM
	from I2C import Adafruit_I2C
from Leg import Leg
from Motor import Motor
import time,math,json,sys,threading
from Queue import Queue,Empty
from MotionPlanning.MotionPlanner import MP

class Robot():

	def __init__(self):

		if RobotUtils.LIVE_TESTING:
			self.pwm = PWM()
			self.pwm.setPWMFreq(RobotUtils.FREQUENCY)
		else:
			self.pwm = None

		self.inputQueue = Queue()
		self.agendaThread = threading.Thread(group=None,target=self.updateAgendaLoop,name="agendaThread")
		self.agendaThread.start()

		self.data_file_name = RobotUtils.DATA_FILE

		self.front_left = None
		self.front_right = None
		self.back_left = None
		self.back_right = None

		self.xMovement 	= 50
		self.yMovement 	= 50

		self.forwardIncMax 	=	15
		self.forwardInc 	=	1
		self.forwardIncMin 	=	1

		self.backwardIncMax 	=	15
		self.backwardInc 		=	1
		self.backwardIncMin 	=	1

		self.movement_threshold  = 3

		self.MotionPlanner = MP()

		self.stop 	  	= False
		self.autonomous = False
		self.last_command = "STAND"
		self.max_command_delay = 200		# Commands have 200 milliseconds to execute before disregarded

		# Horizantal Video Servo Data
		self.horizVidValue 	= 50
		self.horizVidPin = 4
		self.horizVidMinVal = 0
		self.horizVidMaxVal = 100


		# Vertical Video Servo Data
		self.vertVidValue 	= 50
		self.vertVidPin = 8
		self.vertVidMinVal = 0
		self.vertVidMaxVal = 100

		self.horizVidMotor = Motor(self.horizVidValue, self.horizVidPin, self.horizVidMinVal, self.horizVidMaxVal, 0,"horizantal video motor", self.pwm)
		self.vertVidMotor = Motor( self.vertVidValue, self.vertVidPin, self.vertVidMinVal, self.vertVidMaxVal, 0,"vertical video motor", self.pwm)

		self.setup()
		self.reset()
		print '\033[93m' + "Robot Created. this was printed from robot class of number: " +str(id(self))+ '\033[0m'
		self.updateAgenda()


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

		mid_horiz_value 		=  legData["motors"]["middle"]["horizValue"]
		middle_pin 				= legData["motors"]["middle"]["pinValue"]
		middle_min 				= constants["middleRange"]["min"]
		middle_max 				= constants["middleRange"]["max"]
		middle_offset_to_center = constants["midOffsetFromHoriz"]

		leg_horiz_value 		= legData["motors"]["leg"]["horizValue"]
		leg_pin 				= legData["motors"]["leg"]["pinValue"]
		leg_min 				= constants["legRange"]["min"]
		leg_max 				= constants["legRange"]["max"]
		leg_offset_to_center 	= constants["legOffsetFromHoriz"]

		leg = Leg( self.pwm, leg_name, body_pin,	body_min,	body_max,	body_center, mid_horiz_value, 	middle_pin,	middle_min,	middle_max,	middle_offset_to_center, leg_horiz_value, 	leg_pin,	leg_min,	leg_max,	leg_offset_to_center)

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

	def processData(self,data):
		self.xMovement 		= float(data["data"]["xMovement"])
		self.yMovement	 	= float(data["data"]["yMovement"])
		self.horizVidValue 	= float(data["data"]["horizontalVideo"])
		self.vertVidValue 	= float(data["data"]["verticalVideo"])
		self.stop 	  		= data["data"]["stop"]
		self.autonomous 	= data["data"]["autonomous"]

	def updateAgendaLoop(self):
		while True:
			try:
				data = self.inputQueue.get_nowait()
				#if math.fabs( int(data.data["timestamp"]) - int(round(time.time() * 1000))) > self.max_command_delay:
				#	print "Command Delay Exceeded, command SHOULD be ignored"
				
				self.processData(data)
				self.updateAgenda()
			except Empty:
				print "pass"
				pass

			time.sleep(RobotUtils.AGENDA_UPDATE_SPEED)
			
		print '\033[94m' + "Robot: QUEUE READING FINISHED" + '\033[0m'
		sys.exit()

	# acts as central coordinator for the robot - raeads incoming data + state of the bot and calls methods accordingly
	def updateAgenda(self):

		print "in updateAgeda()"
		# if stop is called, the bot freezes in standing pose
		if not self.stop:

			# bot is autonomous, which has not been built yet, so we stand. Or do the mamba.
			if self.autonomous:
				self.reset()
				print '\033[94m' + "Robot: AUTONOMOUS" + '\033[0m'

			# bot is teleop mode
			else:
				# update camera motors
				self.horizVidMotor.moveTo(self.horizVidValue)
				self.vertVidMotor.moveTo(self.vertVidValue)

				xMagnitude = abs(self.xMovement - 50)
				yMagnitude = abs(self.yMovement - 50)

				# filter out value fluctuation by ensuring movment commands are past a certain threshold. Movement commands must be greater than 50 +- threshold to perform a command
				if ( xMagnitude  > self.movement_threshold) or ( yMagnitude  > self.movement_threshold):

					# command to move in the x axis rank higher in importance than command to move in y axis
					if xMagnitude > yMagnitude:

						# if xMovement is greater than 50 than we move left
						if self.xMovement < 50:
							print '\033[95m' + "Robot: LEFT" + '\033[0m \n'
							self.turn(1)
							self.forwardInc = self.forwardIncMin

						# turn left
						elif self.xMovement >= 50:
							self.turn(-1)
							print '\033[95m' + "Robot: RIGHT" + '\033[0m \n'
							self.forwardInc = self.forwardIncMin

						else:
							print "logic error. This should not ever be printed"

					# command to move in the y axis rank higher in importance than command to move in x axis
					else:

						# move forward
						if self.yMovement > 50:

							print '\033[95m' + "Robot: FORWARD" + '\033[0m \n'
							# perform next segment of forward walk
							#self.walkInit()


							# increment forward walk increment variable
							self.forwardInc += 1

							# test to see if forward walk incrment variable has reached maximum value and reset it to min value if it has
							if self.forwardInc == self.forwardIncMax:
								self.forwardInc = self.forwardIncMin

							# reset the backward incrementer, because the backward motion has been stopped, and needs to reset
							self.backwardInc = self.backwardIncMin

						# move backward
						elif self.yMovement <= 50:
							print '\033[95m' + "Robot: BACKWARD" + '\033[0m \n'

						else:
							print "logic error. this should not be printed"

				else:
					print "\033[94m","Robot: STAND","\033[0m"
					if not self.last_command == "STAND":
						self.last_command = "STAND"
						self.reset()

		else:
			print "\033[94m","Robot: STOP \033[0m"
			self.reset()


	# Sets quadbot to standing position
	def reset(self):
		self.front_right.reset()
		self.front_left.reset()	
		self.back_right.reset()
		self.back_left.reset()
		
	# resets legs to default position
	def init_stand(self):
		
		velocity = 5
		increment_count = 100
		
		body_offset = 0
		middle_offset = 100 
		leg_offset = 100
		
		
		print "lunge start"
		self.lunge(body_offset, middle_offset, leg_offset, body_offset, middle_offset, leg_offset, 
					body_offset, middle_offset, leg_offset, body_offset, middle_offset, leg_offset, 
					increment_count,velocity)
		
		time.sleep(2)
		
		self.lunge(body_offset, -middle_offset/4, -leg_offset/2, body_offset, -middle_offset/4, -leg_offset/2, 
					body_offset, -middle_offset/4, -leg_offset/2, body_offset, -middle_offset/4, -leg_offset/2, 
					increment_count,velocity)

		time.sleep(2)
		self.reset()
		
	def testMotionPlanner(self):
		x_min = 6
		x_max = 17
		y_min = -10
		y_max = 13
		
		print "Beggining Tests"

		for x in xrange(x_min,x_max,1):
			y = 0
			values = self.MotionPlanner.calcThetas(x,y)
			print "Moving to: ",values
			self.back_left.middle.moveTo(values[0])
			self.back_left.leg.moveTo(values[1])
			time.sleep(.25)
				
		for y in xrange(y_min,y_max,1):
			x = 13
			values = self.MotionPlanner.calcThetas(x,y)
			print "Moving to: ",values
			self.back_left.middle.moveTo(values[0])
			self.back_left.leg.moveTo(values[1])
			time.sleep(.25)
			
			
		
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

		if(direction > 0):
			turnDegree = 20
		else:
			turnDegree = -20

		stepHeightMid = 60
		stepHeightLeg = 5
		velocity = 0.002
		time_delay = 0
		self.front_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, velocity, time_delay)
		time.sleep(time_delay)

		self.back_left.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, velocity, time_delay)
		time.sleep(time_delay)

		self.front_left.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg,velocity,time_delay)
		time.sleep(time_delay)

		self.back_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, velocity, time_delay)
		time.sleep(time_delay)
		self.reset()


	# Move all motors at once for synchronized motion (lunging)
	def lunge(self, FRB, FRM, FRL, FLB, FLM, FLL, BLB, BLM, BLL, BRB, BRM, BRL, increment_count,velocity):
		
		increment_count = float(increment_count)
		velocity = float(velocity)
		
		for x in range(int(increment_count)):
			
			self.front_right.body.moveOffset(float(FRB)/(increment_count))
			self.front_right.middle.moveOffset(float(FRM)/increment_count)
			self.front_right.leg.moveOffset(float(FRL)/increment_count)

			self.front_left.body.moveOffset(float(FLB)/increment_count)
			self.front_left.middle.moveOffset(float(FLM)/increment_count)
			self.front_left.leg.moveOffset(float(FLL)/increment_count)

			self.back_left.body.moveOffset(float(BLB)/increment_count)
			self.back_left.middle.moveOffset(float(BLM)/increment_count)
			self.back_left.leg.moveOffset(float(BLL)/increment_count)

			self.back_right.body.moveOffset(float(BRB)/increment_count)
			self.back_right.middle.moveOffset(float(BRM)/increment_count)
			self.back_right.leg.moveOffset(float(BRL)/increment_count)

			time.sleep(velocity/increment_count)




	# Method to walk forward. broken into ___ segments, which when executed sequentially create desired gate.
	def forward(self):
		self.reset()
		print "Beggining Walk"
		
		# Walk Methon Inputs
		std_pvt_body_delta = 15;		# std_pvt is short for 'standard pivor step'
		std_pvt_middle_delta = 20;
		std_pvt_leg_delta = 15;		
		
		leg_ext_body_delta = 25;		# leg_ext is short for 'Leg Extend'.
		leg_ext_middle_delta = 20;
		leg_ext_leg_delta = 15;
		leg_ext_middle_raise = 30;
		
		lunge_intensity = 1.0
		lunge_increment_count = 30
		
		L1_FRB_lunge_offset = lunge_intensity * 0						# XXX_lunge_offset provides parameters for lunge method
		L1_FRM_lunge_offset = lunge_intensity * 20
		L1_FRL_lunge_offset = lunge_intensity * -28
		
		L1_FLB_lunge_offset = lunge_intensity * 40
		L1_FLM_lunge_offset = lunge_intensity * 0
		L1_FLL_lunge_offset = lunge_intensity * 0
		
		L1_BLB_lunge_offset = lunge_intensity * 20
		L1_BLM_lunge_offset = lunge_intensity * -10
		L1_BLL_lunge_offset = lunge_intensity * 28
		
		L1_BRB_lunge_offset = lunge_intensity *  -20
		L1_BRM_lunge_offset = lunge_intensity * 0
		L1_BRL_lunge_offset = lunge_intensity * 0

	
		L2_FRB_lunge_offset = -L1_FLB_lunge_offset		# XXX_lunge_offset provides parameters for lunge method
		L2_FRM_lunge_offset = L1_FLM_lunge_offset
		L2_FRL_lunge_offset = L1_FLL_lunge_offset
		
		L2_FLB_lunge_offset = L1_FRB_lunge_offset
		L2_FLM_lunge_offset = 1.25*L1_FRM_lunge_offset
		L2_FLL_lunge_offset = 1.25*L1_FRL_lunge_offset
		
		L2_BLB_lunge_offset = -L1_BRB_lunge_offset
		L2_BLM_lunge_offset = 0
		L2_BLL_lunge_offset = 0
		
		L2_BRB_lunge_offset = -L1_BLB_lunge_offset
		L2_BRM_lunge_offset = L1_BLM_lunge_offset
		L2_BRL_lunge_offset = L1_BLL_lunge_offset
		
		
		
		velocity = .1;					# Time for motor to move to new position. small velocity -> rapid movement
		time_delay = 0;					# Time delay between motor commands in leg method calls
		segment_delay = .1;				# 3 second delay between commands for debugging purposes
		
		# First Segment: back right standard pivot step
		self.back_right.standardPivotStep(	std_pvt_body_delta,	std_pvt_middle_delta,	std_pvt_leg_delta,	velocity/10,	time_delay/10)
		time.sleep(segment_delay)
		
		# Second Segment: front right leg extends
		self.front_right.legExtend( leg_ext_body_delta, leg_ext_middle_delta, leg_ext_leg_delta, leg_ext_middle_raise,velocity, time_delay)
		time.sleep(segment_delay)
		
		# Condense forward and right
		self.lunge(
			L1_FRB_lunge_offset,
			L1_FRM_lunge_offset,
			L1_FRL_lunge_offset,
			L1_FLB_lunge_offset,
			L1_FLM_lunge_offset,
			L1_FLL_lunge_offset,
			L1_BLB_lunge_offset,
			L1_BLM_lunge_offset,
			L1_BLL_lunge_offset,
			L1_BRB_lunge_offset,
			L1_BRM_lunge_offset,
			L1_BRL_lunge_offset,
			lunge_increment_count,
			velocity
		)
		
		time.sleep(segment_delay)

		# Back left standard pivot
		self.back_left.reset()
		self.back_left.standardPivotStep( -std_pvt_body_delta/2,	std_pvt_middle_delta,	std_pvt_leg_delta,	velocity,	time_delay)
		
		time.sleep(segment_delay)
		
		# Front Left Leg Extend body_delta, middle_delta, leg_delta,middle_raise
		self.front_left.reset()
		self.front_left.legExtend( -leg_ext_body_delta, -leg_ext_middle_delta, leg_ext_middle_raise, leg_ext_middle_raise, velocity, time_delay)
		time.sleep(segment_delay)
		
		# Front left condence forward
		self.lunge(
			L2_FRB_lunge_offset,
			L2_FRM_lunge_offset,
			L2_FRL_lunge_offset,
			L2_FLB_lunge_offset,
			L2_FLM_lunge_offset,
			L2_FLL_lunge_offset,
			L2_BLB_lunge_offset,
			L2_BLM_lunge_offset,
			L2_BLL_lunge_offset,
			L2_BRB_lunge_offset,
			L2_BRM_lunge_offset,
			L2_BRL_lunge_offset,
			lunge_increment_count,
			velocity
		)
		
		time.sleep(segment_delay)
		
		# Back left reset
		self.back_right.middle.moveOffSetInT(50,velocity*segment_delay)		
		self.back_right.reset()
		
		time.sleep(segment_delay)
		
		
		
	def legacy_forward(self):
		print "Starting walk"
		velocity = .01
		time_delay = 0
		step_wait = 2;
		std_piv_step_body_delta = -15
		std_piv_step_middle_delta = 50
		std_piv_step_leg_delta = 5

		self.front_left.standardPivotStep(std_piv_step_body_delta, std_piv_step_middle_delta, std_piv_step_leg_delta,velocity,time_delay)
		time.sleep(step_wait)

		self.back_right.standardPivotStep(-std_piv_step_body_delta, std_piv_step_middle_delta, std_piv_step_leg_delta,velocity,time_delay)
		time.sleep(step_wait)

		leg_extend_body_delta 	= 20
		leg_extend_middle_delta = -5
		leg_extend_leg_delta 	= 28

		self.front_right.legExtend( leg_extend_body_delta, leg_extend_middle_delta, leg_extend_leg_delta, velocity, time_delay)
		time.sleep(step_wait)

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
		time.sleep(step_wait)

		# back left standard pivot step with mid offset"
		self.back_left.standardPivotStepWithMidMovement(leg_step_BLbody_delta, leg_step_BLmiddle_delta, leg_step_BLleg_delta,velocity,time_delay)

		leg_step_FRbody_delta = -40
		leg_step_FRmiddle_delta = 5
		leg_step_FRleg_delta = 28

		# front left standard pivot step with mid movement"
		self.front_left.standardPivotStepWithMidMovement(leg_step_FRbody_delta, leg_step_FRmiddle_delta, leg_step_FRleg_delta, velocity,time_delay)
		time.sleep(step_wait)

		frontRightBodySplitDiff = self.front_right.body.center_value - self.front_right.body.value
		frontRightMiddleSplitDiff = self.front_right.middle.value - self.front_right.middle.center_value
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
			self.front_right.middle.moveOffset(1)
			self.front_right.leg.moveOffset(frontRightLegSplitDiff/splitNum)

			#self.front_left.body.moveOffset(frontLeftBodySplitDiff/splitNum)
			self.front_left.middle.moveOffset(frontLeftMiddleSplitDiff/splitNum)
			self.front_left.leg.moveOffset(frontLeftLegSplitDiff/splitNum)

			self.back_right.body.moveOffset(backRightBodySwing)
			self.back_right.middle.moveOffset(backRightMiddleSwing)
			self.back_right.leg.moveOffset(backRightLegSwing)
			self.back_left.body.moveOffset(backLeftBodySwing)

		time.sleep(step_wait)

		leg_step_BRbody_delta = 30
		leg_step_BRmiddle_delta = 30
		leg_step_BRleg_delta = -28

		self.back_right.standardPivotStepWithMidMovement(leg_step_BRbody_delta, leg_step_BRmiddle_delta, leg_step_BRleg_delta,velocity,time_delay)

		leg_extend_body_delta = 35
		leg_extend_middle_delta =-5
		leg_extend_leg_delta = 28

		self.front_right.legExtend( leg_extend_body_delta, leg_extend_middle_delta, leg_extend_leg_delta, velocity, time_delay)
		time.sleep(step_wait)

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
		time.sleep(step_wait)

		self.back_left.standardPivotStepWithMidMovement(leg_step_BLbody_delta, leg_step_BLmiddle_delta, leg_step_BLleg_delta,velocity,time_delay)

		self.front_left.legExtend( -leg_extend_body_delta, leg_extend_middle_delta, leg_extend_leg_delta, velocity, time_delay)
		time.sleep(step_wait)

		LlungeFRbody= -40
		LlungeBLbody= 20
		LlungeFLmiddle = 30
		LlungeFLleg = -28
		LlungeBRmiddle = -10
		LlungeBRleg = 28

		self.lunge(LlungeFRbody, 0,0,0,LlungeFLmiddle,LlungeFLleg, LlungeBLbody,0,0 ,0,LlungeBRmiddle, LlungeBRleg)
		self.reset()
		print "Done with walk."
		time.sleep(10)


