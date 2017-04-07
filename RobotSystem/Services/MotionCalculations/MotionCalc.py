import sys,math,numpy
from RobotSystem.Services.Utilities.RobotUtils import RobotUtils

# MotionPlanner provides target values given a desired end state



# math.cos(X) - return the cosine of x radians
# math.cos(math.radians(X)) - return the cosine of x radians

# 						_ _ _
#					 /  	  \
#	 			  /    / ---   \
#	 R1 /      /    /        \  \
# 		   	 /    /   theta2  \  \   \ R2
#		   /   /               \  \
#        /    /                \  \
#	    (   )                    \  \
#	__ |   /  theta1	    	  |__|
#  ____   |
#

# Input desired x, y cooredinates, output mid, leg angle, or -1

class MotionCalc(object):

    def __init__(self):
        super(MotionPlanner, self).__init__()

        self._R1 = RobotUtils.R1
        self._R2 = RobotUtils.R2

        self.t1Max = 90
        self.t2Max = 180

        self.DEG_TO_RAD = RobotUtil.RobotUtils.DEG_TO_RAD

        self.TARGET_THRESHOLD = .5
        self.THETA_INCREMENT = 1

        self.INSIGNIFICANT_THRESHOLD = 10**-4

    def getX(self,t1,t2):
        retval = (self._R2 * math.sin( self.DEG_TO_RAD * t2 )) + ( self._R1  * math.sin(self.DEG_TO_RAD * t1))
        if (math.fabs(retval) < self.INSIGNIFICANT_THRESHOLD):
            return 0.0
        return retval

    def getY(self,t1,t2):
        retval = ( self._R2 * math.cos( self.DEG_TO_RAD * t2 ) ) - ( self._R1  * math.cos(self.DEG_TO_RAD * t1) )
        if (math.fabs(retval) < self.INSIGNIFICANT_THRESHOLD):
            return 0.0
        return retval

    def getThetaAprox(self,x,y):
        returnThetas = [-1,-1]
        minError = 10**5
        for t2 in xrange(-self.t2Max,self.t2Max,self.THETA_INCREMENT):
            for t1 in xrange(-self.t1Max,self.t1Max,self.THETA_INCREMENT):
                calc_x = self.getX(t1,t2)
                calc_y = self.getY(t1,t2)
                if (  math.fabs(calc_x-x) < self.TARGET_THRESHOLD  ) and (  math.fabs(calc_y-y) < self.TARGET_THRESHOLD  ):
                    error = (calc_x - x)**2 + (calc_y - y)**2
                    if error < minError:
                        minError = error
                        returnThetas = [t1,t2]
                    print "t1, t2: ",t1,", ",t2," error: ",error
        return returnThetas

    def calcDistance(self,x1,x2,y1,y2):
        return math.sqrt( (x1 - x2)**2 + (y1-y2)**2 )

    def calcThetas(self,x,y):

        # Prevent division by 0
        #if x <  self.INSIGNIFICANT_THRESHOLD

        D = self.calcDistance(x,0,y,0)

        t1_numerator = (D**2) - (self._R2**2) - (self._R1**2)
        t1_denominator = - 2.0 * float(self._R2)  *float(self._R1)
        t1_dividor = t1_numerator / t1_denominator

        t2_numerator = (self._R1**2) - (D**2) - (self._R2**2)
        t2_denomonator = -2.0 * self._R2 * D
        t2_dividor = t2_numerator / t2_denomonator

        gamma = math.degrees(math.atan(float(y)/float(x)))

        if not ( (t1_dividor >= -1 ) and (t1_dividor <= 1)):
            raise ValueError('Theta1 Dividor does not exist within {-1,1}: ',t1_dividor)

        if not ( (t2_dividor >= -1 ) and (t2_dividor <= 1)):
            raise ValueError('Theta2 Dividor does not exist within {-1,1}: ',t2_dividor)

        leg = math.degrees(math.acos(t1_dividor))
        middle = math.degrees(math.acos(t2_dividor))
        middle += gamma

        print "leg: ",leg,", "," middle: ",middle

        legVal = RobotUtil.RobotUtils.scale(leg, 0, 180, RobotUtil.RobotUtils.MIN_MOTOR_VALUE, RobotUtil.RobotUtils.MAX_MOTOR_VALUE)
        midVal = RobotUtil.RobotUtils.scale(mid, 0, 180, RobotUtil.RobotUtils.MIN_MOTOR_VALUE, RobotUtil.RobotUtils.MAX_MOTOR_VALUE)

        return [midVal,legVal]

    def getValues(self):
        returnThetas = getThetaAprox()
        xScaled = RobotUtil.RobotUtils.scale( returnThetas[0], returnThetas[1], s )
        return [RobotUtil.RobotUtils.scale(),   RobotUtil.RobotUtils.scale()    ]






#
