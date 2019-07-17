class Velocity_Filter_New():
    lastPosition = []
    lastPositionSet = False
    presentPosition = []
    velocitySet = [[0,0,0]]
    order = 1
    bandWidth = 1 #1m/s default
    lastValid = [0,0,0]
    def __init__(self, order, bandWidth):
        self.order = order
        self.bandWidth = bandWidth
    def updateVelocity(self, x, y, z, timeDiff):
        if(not(self.lastPositionSet)):
            self.lastPosition = [x,y,z]
            self.lastPositionSet = True
        else:
            multiplier = 1000/timeDiff
            self.presentPosition = [x,y,z]
            self.velocitySet.append([(self.presentPosition[0] - self.lastPosition[0])*multiplier,(self.presentPosition[1] - self.lastPosition[1])*multiplier ,(self.presentPosition[2] - self.lastPosition[2])*multiplier])
            self.lastPosition = self.presentPosition
    #Returns the average of the last N number of points in the list
    def calculateFilteredVelocity(self):
        sum = [0,0,0]
        if(len(self.velocitySet) < self.order):
            raise Exception()
        #Applies FIR
        for i in range(self.order):
            sum[0] = sum[0] + (1/self.order)*(self.velocitySet[len(self.velocitySet)-1-i][0])
            sum[1] = sum[1] + (1/self.order)*(self.velocitySet[len(self.velocitySet)-1-i][1])
            sum[2] = sum[2] + (1/self.order)*(self.velocitySet[len(self.velocitySet)-1-i][2])
        #Second item says if the signal makes it through the band pass
        #if(self.checkBandPass()):
       #     return [sum, True]
       # else:
        #    return [sum, False]
        return sum
    '''
    def checkBandPass(self):
        if(len(self.velocitySet)>1):
            magnitudePast = (self.lastValidVel[0]**2 + self.lastValidVel[1]**2 + self.lastValidVel[2]**2)**0.5
            magnitudeCurr = (self.velocitySet[-1][0]**2 + self.velocitySet[-1][1]**2 + self.velocitySet[-1][2]**2)**0.5
            if(abs(magnitudePast - magnitudeCurr) > self.bandWidth):
                return False
            else:
                self.lastValidVel = self.velocitySet[-1]
                return True
        else:
            self.lastValidVel = self.velocitySet[-1]
            '''
            
    def isFilteringAvailable(self):
        if(len(self.velocitySet) < self.order):
            return False
        else:
            return True
    
