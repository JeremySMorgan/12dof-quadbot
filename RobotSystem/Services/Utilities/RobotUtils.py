import datetime 

class RobotUtils(object):

	R1 				= 11.5					# length of curved leg portion of bot
	R2 				= 6.5					# length of junction between middle and leg servos

	DEG_TO_RAD		= 0.0174533				# Constant to convert degrees to radians

	MAX_MOTOR_VALUE = 100					# Maximum possible motor value
	MIN_MOTOR_VALUE = 0						# Minumum possible motor value

	SERVO_MIN		= 130					# Minumum tick count for duty cycle
	SERVO_MAX		= 570 					# Maximum tick count for duty cycle
	FREQUENCY		= 50 					# 50 Hz creates a 20 ms period, which servos operate with
	DATA_FILE	 	= "../ProjectPrism/RobotSystem/Services/MotorCalibration.json" 	# path to data file

	LEG_DEBUG 		= False					# Debug Legs
	MOTOR_DEBUG 	= False					# Debug Motors
	
	LIVE_TESTING	= False					# Dictates whether program is executing on robot or on a development computer
	VIDEO_STEAMING	= False					# Determines whether the computer/Pi streams live video footage
	LASER_ENABLE 	= False					# Enables Laser

	LASER_PIN		= 17					# Which pin laser is connected to
	
	AGENDA_UPDATE_SPEED = 1				# Time delay between polls of new agenda

	TURN_LEFT 		= 0						# Arbitrary constants which are used to commincate moves between the hypervisor and the motion planner
	TURN_RIGHT	 	= 1
	FORWARD 		= 2
	BACKWARD 		= 3
	STOP			= 4
	AUTONOMOUS		= 5
	LASER_ON		= 6
	LASER_OFF		= 7
		
	INVALID_DATA_ERROR = 8

	HORIZONTAL_VID_PIN = 3
	HORIZONTAL_VID_CENTER = 40
	
	VERTICAL_VID_PIN = 	7
	VERTICAL_VID_CENTER = 25
	
	HORIZONTAL_VID_MIN_VAL = 0
	HORIZONTAL_VID_MAX_VAL = 100

	VERTICAL_VID_MIN_VAL = 0
	VERTICAL_VID_MAX_VAL = 100
	

	

	COLORS = {								# Unix codes for special priting
		"HEADER"		: 	{"value":'\033[95m', "code":0},
		"OKBLUE" 		: 	{"value":'\033[94m', "code":1},
		"OKGREEN" 		: 	{"value":'\033[92m', "code":2},
		"WARNING" 		: 	{"value":'\033[93m', "code":3},
		"FAIL" 			: 	{"value":'\033[91m', "code":4},
		"ENDC" 			: 	{"value":'\033[0m', "code":5},
		"BOLD" 			: 	{"value":'\033[1m', "code":6},
		"UNDERLINE" 	: 	{"value":'\033[4m', "code":7},
		"STANDARD"		:	{"value":'', "code":8}
	}

	MIN_MOVEMENT_THRESHOLD = 5

	# takes any numer and returns 1 if positive, -1 if negative, and 0 if input is 0
	@staticmethod
	def PositiveOrNegative(n):
		if (n>0):
			return 1
		elif (n<0):
			return -1
		else:
			return 0

	@staticmethod
	def scale(OldValue, OldMin, OldMax, NewMin, NewMax):
		NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
		return NewValue

	@staticmethod
	def ColorPrinter( caller, message, color):
		# [03/Apr/2017 18:37:10]
		time = datetime.datetime.now()
		prefix = "["+str(time.day)+"/"+str(time.month)+ "/" + str(time.year) + " " + str(time.hour) + ":" + str(time.minute) + ":" + str(time.second) +  " ] "
		prefix = prefix +  RobotUtils.COLORS["BOLD"]["value"]+ RobotUtils.COLORS["UNDERLINE"]["value"]+ caller+ RobotUtils.COLORS["ENDC"]["value"]+ ":"
		print  prefix, RobotUtils.COLORS[color]["value"] ,message , RobotUtils.COLORS["ENDC"]["value"]

