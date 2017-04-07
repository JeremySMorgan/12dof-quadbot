#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time

class MotionController:

    def __init__(
        self,
        TURN_LEFT,
        TURN_RIGHT,
        FORWARD,
        BACKWARD,
        STOP,
        AUTONOMOUS,
        INVALID_DATA_ERROR,
        motors,
        RobotUtils,
        ):
        self.TURN_LEFT = TURN_LEFT
        self.TURN_RIGHT = TURN_RIGHT
        self.FORWARD = FORWARD
        self.BACKWARD = BACKWARD
        self.STOP = STOP
        self.AUTONOMOUS = AUTONOMOUS
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
        else:

            print 'Invalid Command Recieved'

    def updateCameras(self, data):
        horizVidValue = float(data['data']['horizontalVideo'])
        vertVidValue = float(data['data']['verticalVideo'])
        self.horizVidMotor.moveTo(horizVidValue)
        self.vertVidMotor.moveTo(vertVidValue)

    def NextMove(self, data):
        xMovement = float(data['data']['xMovement'])
        yMovement = float(data['data']['yMovement'])
        stop = data['data']['stop']
        autonomous = data['data']['autonomous']

        # Stop has higher precedence than any other command

        if stop:
            return self.STOP

        if autonomous:
            return self.AUTONOMOUS

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
        if direction == 1:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,
                    'Turning Right', 'OKBLUE')
            turnDegree = 20

        elif direction == -1:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,
                    'Turning Left', 'OKBLUE')
            turnDegree = -20

        else:
            self.RobotUtils.ColorPrinter(self.__class__.__name__,
                    'Invalid input to turn command', 'FAIL')
            sys.exit();

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


    def forward(self):
        self.RobotUtils.ColorPrinter(self.__class__.__name__, 'Forward'
                , 'OKBLUE')

    def backward(self):
        self.RobotUtils.ColorPrinter(self.__class__.__name__, 'Backward'
                , 'OKBLUE')

    def stop(self):
        self.RobotUtils.ColorPrinter(self.__class__.__name__, 'Stop'
                , 'OKBLUE')

    def autonomous(self):
        self.RobotUtils.ColorPrinter(self.__class__.__name__, 'Autonomous'
                , 'OKBLUE')

    def reset(self):
        self.front_left.reset()
        self.front_right.reset()
        self.back_left.reset()
        self.back_right.reset()

#
