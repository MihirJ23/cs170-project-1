import numpy
import time

#class structure
class puzzle:
    def __init__(self, matrix): #template w necessary variables
        self.matrix = matrix
        self.parent = None
        self.depth = 0
        self.cost = 0
        self.expended = 0

    #helper functions
    def CheckGoal(goal):
        if puzzle == goal:
            return True

        return False    

    def printMatrix(self):
        for row in self:
            print(row)

    def returnMatrix(self):
        return self

