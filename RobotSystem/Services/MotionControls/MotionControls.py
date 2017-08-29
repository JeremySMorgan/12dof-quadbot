#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import math

class MotionController(object):

    def __init__(
        self,
        TURN_LEFT,
        TURN_RIGHT,
        FORWARD,
        BACKWARD,
        STOP,
        AUTONOMOUS,
        LASER_ON,
        LASER_OFF,
        INVALID_DATA_ERROR,
        motors,
        LaserDriver,
        RobotUtils,
        ):
        self.TURN_LEFT = TURN_LEFT
        self.TURN_RIGHT = TURN_RIGHT
        self.FORWARD = FORWARD
        self.BACKWARD = BACKWARD
        self.STOP = STOP
        self.AUTONOMOUS = AUTONOMOUS
        self.LASER_ON = LASER_ON
        self.LASER_OFF = LASER_OFF
        self.INVALID_DATA_ERROR = INVALID_DATA_ERROR

        self.front_left = motors[0]
        self.front_right = motors[1]
        self.back_left = motors[2]
        self.back_right = motors[3]
        self.horizVidMotor = motors[4]
        self.vertVidMotor = motors[5]

        self.RobotUtils = RobotUtils
        self.MIN_MOVEMENT_THRESHOLD = \
            self.RobotUtils.MIN_MOVEMENT_THRESHOLD
	

        self.LaserDriver = LaserDriver
	
        self.forwardSide = 0						# Variable which stores which side should lead in walking method
        self.stopped = True							# Stores whether the bot is stopped. Unneccessary stop commands causes shaking

    def MakeMove(self, move):
        if move == self.TURN_LEFT:
            self.turn(-1)
        elif move == self.TURN_RIGHT:

            self.turn(1)
        elif move == self.FORWARD:

            self.forward()
        elif move == self.BACKWARD:

            self.backward()
        elif move == self.AUTONOMOUS:

            self.autonomous()
        elif move == self.STOP:
            self.stop()
            
        elif move == self.LASER_ON:
            self.LaserOn()
	
        elif move == self.LASER_OFF:
            self.LaserOff()  
        
        else:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,
                            'Invalid Data Recieved in MakeMove()'
                            , 'FAIL')

    def LaserOn(self):
        self.LaserDriver.On()
	
    def LaserOff(self):
        self.LaserDriver.Off()

    def updateCameras(self, data):
        horizVidValue = self.RobotUtils.HORIZONTAL_VID_CENTER + (int(float(data['data']['horizontalVideo'])))-50
        vertVidValue = self.RobotUtils.VERTICAL_VID_CENTER + (int(float(data['data']['verticalVideo'])))-50 

        self.horizVidMotor.moveTo(horizVidValue)
        self.vertVidMotor.moveTo(vertVidValue)

    def NextMove(self, data):
        xMovement = float(data['data']['xMovement'])
        yMovement = float(data['data']['yMovement'])
        stop = data['data']['stop']
        autonomous = data['data']['autonomous']
        laser = data['data']['laser']

        # Stop has higher precedence than any other command

        if stop:
            return self.STOP

        if autonomous:
            return self.AUTONOMOUS
		
        if self.LaserDriver.firing != laser:
            if laser:
                return self.LASER_ON
            else:
                return self.LASER_OFF

        #                   | y == forwards
        #                   |
        #                   |
        #  -x == left       |                x == right
        #  <------------------------------------>
        #                   |
        #                   |
        #                   | -y == backwards

        # magnitude is the intensity of the command, i.e. the distance the value is from 50 (baseline)

        xMagnitude = abs(xMovement - 50)
        yMagnitude = abs(yMovement - 50)

        # filter out value fluctuation by ensuring movment commands are past a certain threshold. Movement commands must be greater than 50 +- threshold to perform a command

        if xMagnitude > self.MIN_MOVEMENT_THRESHOLD or yMagnitude \
            > self.MIN_MOVEMENT_THRESHOLD:

            # command to move in the x axis rank higher in importance than command to move in y axis
            if xMagnitude > yMagnitude:
                # if xMovement is greater than 50 than we move left
                if xMovement < 50:
                    return self.TURN_LEFT

                elif xMovement >= 50:
                    return self.TURN_RIGHT
                else:
                    self.RobotUtils.ColorPrinter(self.__class__.__name__,
                            'Invalid Data Recieved from xMagnitude > yMagnitude branch of NextMove()'
                            , 'FAIL')
                return self.INVALID_DATA_ERROR
            elif yMagnitude > xMagnitude:

            # command to move in the y axis rank higher in importance than command to move in x axis

                # move forward
                if yMovement > 50:
                    return self.FORWARD
                elif yMovement <= 50:
                    return self.BACKWARD
                else:
                    self.RobotUtils.ColorPrinter(self.__class__.__name__,
                            'Invalid Data Recieved from yMagnitude > xMagnitude branch of NextMove()'
                            , 'FAIL')
                    return self.INVALID_DATA_ERROR
        else:
            return self.STOP

    # ColorPrinter( caller, message, color):

    def turn(self, direction):
		
        self.stopped = False
		
        turnDegree = -30
        if direction == -1:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,'Turning Right', 'OKBLUE')
        elif direction == 1:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,'Turning Left', 'OKBLUE')
            turnDegree *= -1
        else:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,
                    'Invalid input to turn command', 'FAIL')
            sys.exit();

        stepHeightMid = 60
        stepHeightLeg = 5
        motion_increment_delay = 0.005
        time_delay = 1/45.0
        
        self.front_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, time_delay)
        time.sleep(time_delay)

        self.back_left.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, 0, 0)
        time.sleep(time_delay)

        self.front_left.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg,motion_increment_delay,time_delay)
        time.sleep(time_delay)

        self.back_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, time_delay)
        time.sleep(time_delay)
        self.reset()

    def lunge(self, FRB, FRM, FRL, FLB, FLM, FLL, BLB, BLM, BLL, BRB, BRM, BRL, increment_count, movement_delay):
	
		splitNum = float(increment_count)

		for x in range(int(splitNum)):
			
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
			
			time.sleep(movement_delay)


    def forward(self):
		self.stopped = False

		self.RobotUtils.ColorPrinter(self.__class__.__name__, 'Forward', 'OKBLUE')

		# Motion Variables
		turnDegree = 15
		stepHeightMid = 60
		stepHeightLeg = 15
		lungeTurnDeg = turnDegree

		lunge_increment_delay = 0
		lunge_increment_count= 50
		motion_increment_delay = 0
		mid_step_time_delay = 0
		
		pre_lunge_motion_increment_delay = 0
		pre_lunge_mid_step_time_delay = 0
		
		post_lunge_motion_increment_delay = 0
		post_lunge_mid_step_time_delay = 0
		
		step_time_delay = .05
		end_delay = 2

		if self.forwardSide == 0:

			self.back_left.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.front_left.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.back_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.front_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.forwardSide = 1
		
		else:

			self.back_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.front_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.back_left.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.front_left.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)
			self.forwardSide = 0
		
        
		time.sleep(end_delay)
		self.reset()
		time.sleep(end_delay)
		
    def backward(self):
		self.stopped = False
		self.RobotUtils.ColorPrinter(self.__class__.__name__, 'in MotionController.Backward', 'OKBLUE')

		# Motion Variables
		turnDegree = 20
		stepHeightMid = 60
		stepHeightLeg = 15
		lungeTurnDeg = turnDegree

		lunge_increment_delay = .001
		lunge_increment_count= 50
		
		pre_lunge_motion_increment_delay = 0
		pre_lunge_mid_step_time_delay = 0
		
		post_lunge_motion_increment_delay = 0
		post_lunge_mid_step_time_delay = 0
		
		step_time_delay = .5
		end_delay = .5

		# Backs forward
		self.front_left.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, pre_lunge_motion_increment_delay, pre_lunge_mid_step_time_delay)

		time.sleep(step_time_delay)
		self.front_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, 0, 0)
		
		time.sleep(step_time_delay)
		self.lunge(lungeTurnDeg,0,0,lungeTurnDeg,0,0,lungeTurnDeg,0,0,-lungeTurnDeg,0,0,lunge_increment_count,lunge_increment_delay)
		time.sleep(step_time_delay)
		
		# front push forward
		if (self.forwardSide==0):
			self.back_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, post_lunge_motion_increment_delay, post_lunge_mid_step_time_delay)
			time.sleep(step_time_delay)
						
			self.back_left.standardPivotStep(-turnDegree, stepHeightMid*2, stepHeightLeg, post_lunge_motion_increment_delay, post_lunge_mid_step_time_delay)
			time.sleep(step_time_delay)

			self.forwardSide = 1
		
		else:			
			self.back_left.standardPivotStep(-turnDegree, stepHeightMid*2, stepHeightLeg, post_lunge_motion_increment_delay, post_lunge_mid_step_time_delay)
			time.sleep(step_time_delay)

			self.back_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, post_lunge_motion_increment_delay, post_lunge_mid_step_time_delay)
			time.sleep(step_time_delay)

			self.forwardSide = 0
		
		time.sleep(end_delay)
		
		self.reset()


    def stop(self):
        if not self.stopped:
            self.reset()
            self.stopped = True
            self.RobotUtils.ColorPrinter(self.__class__.__name__, 'Stop', 'OKBLUE')
        

    def autonomous(self):
        if not self.stopped:
            self.reset()
            self.stopped = True
            self.RobotUtils.ColorPrinter(self.__class__.__name__, 'Autonomous', 'OKBLUE')

    def reset(self):
        self.front_left.reset()
        self.front_right.reset()
        self.back_left.reset()
        self.back_right.reset()



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


# old forward command

'''
		self.stopped = False

		self.RobotUtils.ColorPrinter(self.__class__.__name__, 'Forward', 'OKBLUE')

		# Motion Variables
		turnDegree = 15
		stepHeightMid = 60
		stepHeightLeg = 15
		lungeTurnDeg = turnDegree

		lunge_increment_delay = 0
		lunge_increment_count= 50
		motion_increment_delay = 0
		mid_step_time_delay = 0
		
		pre_lunge_motion_increment_delay = 0
		pre_lunge_mid_step_time_delay = 0
		
		post_lunge_motion_increment_delay = 0
		post_lunge_mid_step_time_delay = 0
		
		step_time_delay = .05
		end_delay = 2

		if self.forwardSide == 0:

			self.back_left.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.front_left.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.back_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.front_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.forwardSide = 1
		
		else:

			self.back_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.front_right.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.back_left.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

			self.front_left.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, motion_increment_delay, mid_step_time_delay)
			time.sleep(step_time_delay)

            self.forwardSide = 0
		
        
		time.sleep(end_delay)
		self.reset()
		time.sleep(end_delay)
'''


# Non elegent forward gate. sorta works
'''
self.stopped = False

		self.RobotUtils.ColorPrinter(self.__class__.__name__, 'Forward', 'OKBLUE')

		# Motion Variables
		turnDegree = -20
		stepHeightMid = 60
		stepHeightLeg = 15
		lungeTurnDeg = turnDegree

		lunge_increment_delay = .001
		lunge_increment_count= 50
		
		pre_lunge_motion_increment_delay = .01
		pre_lunge_mid_step_time_delay = .01
		
		post_lunge_motion_increment_delay = .005
		post_lunge_mid_step_time_delay = .005
		
		step_time_delay = .02
		end_delay = .5

		# Backs forward
		self.back_left.standardPivotStep(turnDegree, stepHeightMid, stepHeightLeg, pre_lunge_motion_increment_delay, pre_lunge_mid_step_time_delay)
		time.sleep(step_time_delay)

		self.back_right.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, 0, 0)
		time.sleep(step_time_delay)
		
		self.lunge(lungeTurnDeg,0,0,-lungeTurnDeg,0,0,-lungeTurnDeg,0,0,lungeTurnDeg,0,0,lunge_increment_count,lunge_increment_delay)
		
		time.sleep(step_time_delay)
		
		# front push forward
				
		if (self.forwardSide==0):
			self.front_right.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, post_lunge_motion_increment_delay, post_lunge_mid_step_time_delay)
			time.sleep(step_time_delay)
						
			self.front_left.standardPivotStep(turnDegree, stepHeightMid*2, stepHeightLeg, post_lunge_motion_increment_delay, post_lunge_mid_step_time_delay)
			time.sleep(step_time_delay)

			self.forwardSide = 1
		
		else:			
			self.front_left.standardPivotStep(turnDegree, stepHeightMid*2, stepHeightLeg, post_lunge_motion_increment_delay, post_lunge_mid_step_time_delay)
			time.sleep(step_time_delay)

			self.front_right.standardPivotStep(-turnDegree, stepHeightMid, stepHeightLeg, post_lunge_motion_increment_delay, post_lunge_mid_step_time_delay)
			time.sleep(step_time_delay)

			self.forwardSide = 0
		
		time.sleep(end_delay)
		
		self.reset()

'''
