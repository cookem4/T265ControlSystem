
class Position_Filter():
    #List that contains the original positions
    originalPositions = []
    order = 1
    def __init__(self, order):
        self.order = order

    def updateOriginalPosition(self, x, y, z):
        self.originalPositions.append([x, y, z])

    #Returns the average of the last N number of points in the list
    def calculateFilteredPosition(self):
        sum = [0,0,0]
        if(len(self.originalPositions) < self.order):
            raise Exception()
        for i in range(self.order):
            sum[0] = sum[0] + (1/self.order)*(self.originalPositions[len(self.originalPositions)-1-i][0])
            sum[1] = sum[1] + (1/self.order)*(self.originalPositions[len(self.originalPositions)-1-i][1])
            sum[2] = sum[2] + (1/self.order)*(self.originalPositions[len(self.originalPositions)-1-i][2])
        return sum
    #Position filtering is only available after N measurements where N is the order of the filter
    def isFilteringAvailable(self):
        if(len(self.originalPositions) < self.order):
            return False
        else:
            return True
    
