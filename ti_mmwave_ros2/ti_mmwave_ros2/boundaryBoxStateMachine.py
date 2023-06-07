from os.path import exists
from os import mkdir

NO_PRESENCE_MOTION_STATE = 0
MINOR_PRESENCE_MOTION_STATE = 1
MAJOR_PRESENCE_MOTION_STATE = 2

# The minorBoundaryBoxStateMachineType can technically support both the major motion 
# and the minor motion configs. The arguments are exactly the same with the exception
# of major2minorThre and minor2emptyThre. In this implementation, since major mode and
# minor mode are not supported simultaneously, the major2minorThre transitions the
# state machine from major mode to empty mode.

class minorBoundaryBoxStateMachineType:
        
    def __init__(self):
        try:
            self.currentState = NO_PRESENCE_MOTION_STATE
            self.nextState = NO_PRESENCE_MOTION_STATE
            self.minorToEmptyCounter = 0
            self.minorPointThre1 = 0
            self.minorPointThre2 = 0
            self.minorSNRThre2 = 0
            self.minorPointHistThre1 = 0
            self.minorPointHistThre2 = 0
            self.minorSNRHistThre2 = 0
            self.minorHistBufferSize = 0
            self.minorToEmptyThre = 0
            self.xMin = 0
            self.yMin = 0
            self.zMin = 0
            self.xMax = 0
            self.yMax = 0
            self.zMax = 0
            self.history = []
            self.BoundaryBoxIndex = 0
        except:
            print("Boundary Box initialization failed")
    def getState(self):
        return self.currentState

    def configure(self, minorPointThre1, minorPointThre2, minorSNRThre2, minorPointHistThre1, minorPointHistThre2, \
        minorSNRHistThre2, minorHistBufferSize, minorToEmptyThre, xMin, xMax, yMin, yMax, zMin, zMax, BoundaryBoxIndex):
        try:
            self.minorPointThre1 = minorPointThre1
            self.minorPointThre2 = minorPointThre2
            self.minorSNRThre2 = minorSNRThre2
            self.minorPointHistThre1 = minorPointHistThre1
            self.minorPointHistThre2 = minorPointHistThre2
            self.minorSNRHistThre2 = minorSNRHistThre2
            self.minorHistBufferSize = minorHistBufferSize
            self.minorToEmptyThre = minorToEmptyThre
            self.xMin = xMin
            self.xMax = xMax
            self.yMin = yMin
            self.yMax = yMax
            self.zMin = zMin
            self.zMax = zMax
            self.BoundaryBoxIndex = BoundaryBoxIndex
        except:
            print("Minor Boundary Box initialization failed")
    def step(self, clusters):
        totalSNR = 0
        totalnumPoints = 0
        for cluster in clusters:
            if (cluster['x'] > self.xMin and cluster['x'] < self.xMax and cluster['y']> self.yMin \
                and cluster['y'] < self.yMax and cluster['z'] > self.zMin and cluster['z'] < self.zMax):
                totalSNR = totalSNR + cluster['snr']
                totalnumPoints = totalnumPoints + cluster['numPoints']
        
        self.history.append([totalnumPoints, totalSNR])

        if(len(self.history) > self.minorHistBufferSize):
            self.history.pop(0)
    
        self.compute_state()
    
    def compute_state(self):
        if (self.currentState == NO_PRESENCE_MOTION_STATE):
            if(self.history[-1][0] >= self.minorPointThre1):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.minorToEmptyCounter = 0
            elif(self.history[-1][0] >= self.minorPointThre2 and self.history[-1][1] >= self.minorSNRThre2):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.minorToEmptyCounter = 0
            elif(sum(self.history[:][0]) >= self.minorPointHistThre1):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.minorToEmptyCounter = 0
            elif(sum(self.history[:][0]) >= self.minorPointHistThre2 and sum(self.history[:][1]) >= self.minorSNRHistThre2):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.minorToEmptyCounter = 0
        elif(self.currentState == MINOR_PRESENCE_MOTION_STATE):
            if(self.history[-1][0] >= self.minorPointThre1):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.minorToEmptyCounter = 0
            elif(self.history[-1][0] >= self.minorPointThre2 and self.history[-1][1] >= self.minorSNRThre2):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.minorToEmptyCounter = 0
            elif(sum(self.history[:][0]) >= self.minorPointHistThre1):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.minorToEmptyCounter = 0
            elif(sum(self.history[:][0]) >= self.minorPointHistThre2 and sum(self.history[:][1]) >= self.minorSNRHistThre2):
                self.nextState = MINOR_PRESENCE_MOTION_STATE
                self.minorToEmptyCounter = 0
            elif(self.minorToEmptyCounter == self.minorToEmptyThre):
                self.nextState = NO_PRESENCE_MOTION_STATE
            else:
                self.minorToEmptyCounter = self.minorToEmptyCounter + 1
        
        self.currentState = self.nextState

    def getBoundaryBoxIndex(self):
        return self.BoundaryBoxIndex
