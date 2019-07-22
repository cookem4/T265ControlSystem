#This is a newly designed velocity filter with a low pass filter of order n and thresholding
class Velocity_Filter_New():
    lastPosition = []
    lastPositionSet = False
    presentPosition = []
    velocitySet = [[0,0,0]] #Stores all velocities
    order = 1
    thresholdVal = 1 #1m/s default
    lastValid = [0,0,0]
    def __init__(self, order, thresholdVal):
        self.order = order
        self.thresholdVal = thresholdVal
    def updateVelocity(self, x, y, z, timeDiff):
        if(not(self.lastPositionSet)):
            self.lastPosition = [x,y,z]
            self.lastPositionSet = True
        else:
            multiplier = 1000000/timeDiff #Finds frequency based on period in microseconds
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
        return sum
    #Filtering is only available if there are n number of mesurements where n is the order
    def isFilteringAvailable(self):
        if(len(self.velocitySet) < self.order):
            return False
        else:
            return True
    
