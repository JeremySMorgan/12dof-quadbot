#!/usr/bin/python
from Services import *
import time
import math
import json
import sys
import threading
import os
import datetime

class Hypervisor():

    def __init__(self):

        if RobotUtils.LIVE_TESTING:
            self.pwm = PWM()
            self.pwm.setPWMFreq(RobotUtils.FREQUENCY)
        else:
            self.pwm = None

        self.current_data = None
        
        self.agendaThreadAlive = True
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
        self.LASER_ON = RobotUtils.LASER_ON
        self.LASER_OFF = RobotUtils.LASER_OFF
        self.INVALID_DATA_ERROR = RobotUtils.INVALID_DATA_ERROR

        # Video Motors
        self.horizVidMotor = Motor(RobotUtils.HORIZONTAL_VID_CENTER, RobotUtils.HORIZONTAL_VID_PIN, RobotUtils.HORIZONTAL_VID_MIN_VAL, RobotUtils.HORIZONTAL_VID_MAX_VAL, 0, "horizontal video motor", self.pwm)
        self.vertVidMotor = Motor( RobotUtils.VERTICAL_VID_CENTER, RobotUtils.VERTICAL_VID_PIN, RobotUtils.VERTICAL_VID_MIN_VAL, RobotUtils.VERTICAL_VID_MAX_VAL, 0, "vertical video motor", self.pwm)

        # Initiate motors
        self.initiate_motors()
        self.motors = [self.front_left, self.front_right,self.back_left,self.back_right, self.horizVidMotor, self.vertVidMotor ]
        
        # Laser Setup
        laser_enable = RobotUtils.LASER_ENABLE and RobotUtils.LIVE_TESTING
        self.LaserDriver = LaserDriver(RobotUtils.LASER_PIN,laser_enable,'OKGREEN')

        # Create Motion Controller
        self.MotionController = MotionController(self.TURN_LEFT,  self.TURN_RIGHT, self.FORWARD, self.BACKWARD, self.STOP,self.AUTONOMOUS,self.LASER_ON, self.LASER_OFF, self.INVALID_DATA_ERROR, self.motors, self.LaserDriver,RobotUtils)

        RobotUtils.ColorPrinter(self.__class__.__name__, '__init__() finished. Robot Created with id ' +str(id(self)), 'OKBLUE')

    def testSuite(self,operation):

        sleep_time_between_same_motions = 2
        sleep_time_between_different_motions = 5
        test_count = 5

        if operation == "TURN":
            RobotUtils.ColorPrinter(self.__class__.__name__, "In testSuite(). Testing RIGHT turn command", 'OKGREEN')
            for i in range(test_count):
                self.MotionController.turn(1)
                time.sleep(sleep_time_between_same_motions)

            time.sleep(sleep_time_between_different_motions)

            RobotUtils.ColorPrinter(self.__class__.__name__, "In testSuite(). Testing LEFT turn command", 'OKGREEN')
            for i in range(test_count):
                self.MotionController.turn(-1)
                time.sleep(sleep_time_between_same_motions)

        elif operation == "FORWARD":
            RobotUtils.ColorPrinter(self.__class__.__name__, "In testSuite(). Testing FORWARD command", 'OKGREEN')
            
            for i in range(test_count):
                self.MotionController.forward()
                time.sleep(sleep_time_between_same_motions)
        
        elif operation == "BACKWARD":
            RobotUtils.ColorPrinter(self.__class__.__name__, "In testSuite(). Testing FORWARD command", 'OKGREEN')
            
            for i in range(test_count):
                self.MotionController.forward()
                time.sleep(sleep_time_between_same_motions)

        else:
            RobotUtils.ColorPrinter(self.__class__.__name__, "Invalid test suite input", 'FAIL')


    # loads json data and creates Leg objects with add_leg()
    def initiate_motors(self):
        with open(self.data_file_name) as data_file:
            data = json.load(data_file)
            constants = data["constants"]
            for i in range(len(data["legs"])):
                self.add_leg(data["legs"][i],constants)

    # reads dictuanary values from input, creates a Leg object, and adds it to leg variables
    def add_leg(self,legData,constants):

        leg_name = legData["name"]

        body_pin                 = legData["motors"]["body"]["pinValue"]
        body_offset             = legData["motors"]["body"]["offset"]
        body_center             = constants["bodyCenterValue"] + body_offset
        body_min                 = constants["bodyRange"]["min"]
        body_max                 = constants["bodyRange"]["max"]

        mid_horiz_value         = legData["motors"]["middle"]["horizValue"]
        middle_pin                 = legData["motors"]["middle"]["pinValue"]
        middle_min                 = constants["middleRange"]["min"]
        middle_max                 = constants["middleRange"]["max"]
        middle_offset_to_center = constants["midOffsetFromHoriz"]

        leg_horiz_value         = legData["motors"]["leg"]["horizValue"]
        leg_pin                 = legData["motors"]["leg"]["pinValue"]
        leg_min                 = constants["legRange"]["min"]
        leg_max                 = constants["legRange"]["max"]
        leg_offset_to_center     = constants["legOffsetFromHoriz"]

        leg = Leg( self.pwm, leg_name, body_pin,    body_min,    body_max,    body_center, mid_horiz_value,     middle_pin,    middle_min,    middle_max,    middle_offset_to_center, leg_horiz_value, leg_pin, leg_min,    leg_max, leg_offset_to_center)

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

    def stand(self):
        self.MotionController.reset()

    # Called by server when a change in user data is detected
    def inputData(self,data):
        self.current_data = data

    # Ends agenda loop thread
    def endHypervisor(self):
        RobotUtils.ColorPrinter(self.__class__.__name__,'Ending Agenda Thread', 'FAIL')
        self.agendaThreadAlive = False

    def updateAgendaLoop(self):          
        while True:
            if not self.agendaThreadAlive:
                print "trying to shutdown thread"
                self.agendaThread._Thread_stop()
                print "attepy to shutdown thread complete"
            else:                    
                data = self.current_data
                self.updateAgenda(data)

            time.sleep(RobotUtils.AGENDA_UPDATE_SPEED)

    # acts as central coordinator for the robot - raeads incoming data + state of the bot and calls methods accordingly
    def updateAgenda(self,data):
        if data == None:
            if hasattr(self, 'MotionController'):            # If the motion controller has been created, then stand
                self.stand()
            else:
                pass
        else:
            self.MotionController.updateCameras(data)
            nextMove = self.MotionController.NextMove(data)
            if nextMove == self.INVALID_DATA_ERROR:
                RobotUtils.ColorPrinter(self.__class__.__name__,'Invalid command returned by MotionPlanner', 'FAIL')
            else:
                self.MotionController.MakeMove(nextMove)
