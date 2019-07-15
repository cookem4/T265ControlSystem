import math
import numpy as np
from scipy import signal
from Velocity_Filter import Velocity_Filter
from Position_Filter import Position_Filter
from Velocity_Filter_New import Velocity_Filter_New
class Sensor:
    ##FailSafe Related##
    trackLostCounter = []
    FailSafeLostCounts = 10
    FAILSAFE_FLAG = False # INTERFACE ATTRIBUTE

    #FeedBack Related##
    initFlag = False
    initLeaderPosition = [0, 0, 0]
    Position = [] # INTERFACE ATTRIBUTE. A list of lists; each of sublist contains the position of a copter in the world frame. Initializes based on camera measurements.
    Velocity = [] # INTERFACE ATTRIBUTE. Contains all velocity vectors of all copters. List of lists. Initializes at zeros.
    tempVel = []

    
    ##Yaw Related##
    initYaw = []
    yaw = []
    yawCounter = []
    yawFiltered = [] #INTERFACE ATTRIBUTE. Initializes based on camera measuremnts.
    
    ##Filter related##
    firPosFilter = Position_Filter(7) #7 is the max order that minimizes phase delay to around 10ms requrement

    firVelFilter = Velocity_Filter_New(5,0.1)
    #firFilter0 = Velocity_Filter(order = 3, cutoff = 0.1, channelNum = 3) #"Numerical derivation and filtering of the camera measurements"
    #Modifying filter value from 0.1m/s
    nonlinFilterThreshold =  0.5 # m/s

    def __init__(self, numCopters):
        for i in range (numCopters):
            self.Position.append([]) # Positions will be initialized with the actual measurements later.
            self.Velocity.append([0,0,0]) # Velocities initialized at 0s.
            self.tempVel.append([0,0,0])
            self.yaw.append(0)
            self.yawCounter.append(0) # Yaw conters initialized at 0s.
            self.yawFiltered.append(0) # Yaw will be initialized with the actual measurements later.
            self.trackLostCounter.append(0)
            self.firPosFilter = Position_Filter(7)
            
    def failSafe(self, trackingFlag): # This is a list of tracking indexes
        for i in range (len(trackingFlag)):
            if (trackingFlag[i] == False):
                self.trackLostCounter[i] += 1
                if(self.trackLostCounter[i] > self.FailSafeLostCounts):
                    self.FAILSAFE_FLAG = True
            else:
                self.trackLostCounter[i] = 0

    def find_yaw(self, orientations, trackingFlag): # The last element of orientation is the scalar one. List of lists
            "Extraction of the copter's yaw wrt the world frame from the camera measured quaternions"
            for i in range (len(orientations)):
                if (trackingFlag[i] == True):
                    x = orientations[i][0]
                    y = orientations[i][1]
                    z = orientations[i][2]
                    w = orientations[i][3]

                    '''
                    r11 = 1 - 2 * (q2**2 + q3**2)
                    r31 = 2 * (q1 * q3 - q0 * q2)   
                    yaw = math.atan2(-r31,r11)
                    '''
                    
                    yaw  = math.atan2(2*y*w + 2*x*z, 1 - 2*y*y - 2*z*z)
                    roll = math.atan2(2*x*w + 2*y*z, 1 - 2*x*x - 2*z*z)
                    pitch   = math.asin(2*x*y + 2*z*w)
                    
                    '''
                    print("Roll")
                    print(roll*180/math.pi)
                    print("Pitch")
                    print(pitch*180/math.pi)
                    print("Yaw")
                    print(yaw*180/math.pi)
                    print("===========================================")
                    '''
                    #Re-assign since the yaw is actually the pitch
                    yaw = math.atan2(2*x*w + 2*y*z, 1 - 2*x*x - 2*z*z)
                    
                    # initializing yawFiltered
                    if (self.initFlag == False):
                            self.yawFiltered[i] = yaw
                    
                    # Compensating for yaw overflows
                    if (yaw - self.yaw[i] < -5.25): # approx -300 degrees
                            self.yawCounter[i] += 1
                    elif (yaw - self.yaw[i] > 5.25):
                            self.yawCounter[i] += -1
           
                    self.yaw[i] = yaw
                    yaw = yaw + 2 * math.pi * self.yawCounter[i]

                    #Filtering possible spikes
                    #Original value of 0.05
                    if(abs(yaw - self.yawFiltered[i]) < 1):
                            self.yawFiltered[i] = yaw
                        
    def setPosition(self, Position, trackingFlag): #Sets all positions by accounting for the initial position offset. Leader is placed in the origin of the world frame!
        for i in range (len(Position)):
            if (trackingFlag[i] == True):
                pos = Position[i]
                self.firPosFilter.updateOriginalPosition(pos[0], pos[1], pos[2])
                if(self.firPosFilter.isFilteringAvailable()):
                    pos = self.firPosFilter.calculateFilteredPosition()
                self.Position[i] = pos
    def estimateVel(self, timeDiff):
        self.firVelFilter.updateVelocity(self.Position[0][0], self.Position[0][1], self.Position[0][2], timeDiff)
        velocity = self.firVelFilter.velocitySet[-1]
        if(self.firVelFilter.isFilteringAvailable()):
            velocity = self.firVelFilter.calculateFilteredVelocity()
        self.Velocity[0] = velocity
        '''
        for i in range (len(self.Position)):
            if(not(self.pastVelSet)):
                self.Velocity[i] = tempVel
                self.lastVel = tempVel
                self.pastVelSet = True
            else:
                currVelMag = (self.tempVel[i][0]**2 + self.tempVel[i][1]**2  + self.tempVel[i][2]**2 )**0.5
                pastVelMag = (self.lastVel[0]**2  + self.lastVel[1]**2  + self.lastVel[2]**2 )**0.5
                if (abs(currVelMag - pastVelMag) < self.nonlinFilterThreshold):
                    #Pass the bandpass filter
                    self.Velocity[i] = tempVel
                    self.lastVel = tempVel
                else:
                    self.Velocity[i] = self.lastVel
        '''
                        
    def process(self, Position, Orientation, trackingFlag, timeDiff): # Position and Orientation are lists of lists and trackingFlag is a list.
        #Initializing measurements
        if (self.initFlag == False):
            allTracked = True
            for i in range (len(trackingFlag)):
                allTracked = allTracked * trackingFlag[i]
            if (allTracked == True):
                self.initLeaderPosition = Position[0]    #Reading the leader initial position
                self.setPosition(Position, trackingFlag) #Initializing positions
                self.find_yaw(Orientation, trackingFlag) #Initializing yaws
                self.initFlag = True
                print("Copter position/orientation coordinates initialized")
        else:
            self.find_yaw(Orientation, trackingFlag) #self.yawFiltered gets updated only for the copters that have been tracked
            self.setPosition(Position, trackingFlag) #self.Position gets updated only for the copters that have been tracked
            self.estimateVel(timeDiff)                       #self.Velocity gets updated
            self.failSafe(trackingFlag) #If any of the copters is lost for more than 10 consecutive samples, the system goes into a failsafe mode.
