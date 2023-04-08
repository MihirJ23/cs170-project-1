from project1header import *
import numpy
import pandas
import random
import math
import time

startingPoint = 0.0
endingPoint = 0.0

#preset matrices for various depths to test (number at end is depth number)
puzzleGoatState0=numpy.array([[1,2,3], [4,5,6], [7,8,0]])
puzzleGoatState2=numpy.array([[1,2,3], [4,5,6], [0,7,8]])
puzzleGoatState4=numpy.array([[1,2,3], [5,0,6], [4,7,8]])
puzzleGoatState8=numpy.array([[1,3,6], [5,0,2], [4,7,8]])
puzzleGoatState16=numpy.array([[1,6,7], [5,0,3], [4,8,2]])
puzzleGoatState20=numpy.array([[7,1,2], [4,8,5], [6,3,0]])
puzzleGoatState24=numpy.array([[0,7,2], [4,6,1], [3,5,8]])
puzzleGoatStateX=numpy.array([[4,1,2], [5,3,0], [7,8,6]])
puzzleGoalStateDefault=numpy.array([[1,2,3], [4,5,6], [7,8,0]]) #goal state

#heuristic functions
#manhattan distance heuristic
#referred to this link for the algorithm:
#https://stackoverflow.com/questions/16318757/calculating-manhattan-distance-in-python-in-an-8-puzzle-game
def ManhattanDistanceCount(puzzle, goal):
    distance = 0 #manhattan distance variable
    oldRow, oldColumn = 0, 0
    goalRow, goalColumn = 0, 0

    for l in range(1, 9): #1-9 as it is a 3x3 matrix
        for i in range(len(puzzle)): #double for loop to iterate through 3d matrix
            for j in range(len(puzzle)):
                if int(puzzle[i][j]) == l:
                    oldRow = i
                    oldColumn = j
                if goal[i][j] == l: #once 
                    goalRow = i
                    goalColumn = j
        distance += abs(goalRow - oldRow) + abs(goalColumn - oldColumn) 
        #formula for finding manhattan distance

    return distance

#misplaced tile heuristic
#referred to this link 
#https://stackoverflow.com/questions/55030248/manhattan-and-misplaced-heuristic
def MisplacedTileCount(puzzle, goal):
    tileCount = 0 

    for i in range(len(puzzle)): #iterating through 2D matrix
        for j in range(len(puzzle)):
            if int(puzzle[i][j]) != goal[i][j] and int(puzzle[i][j]) != 0: 
                #checks if tiles are in same location as goal matrix
                tileCount += 1 #adds one to h(n) if conditions are met
    
    return tileCount

#algorithm functions
#used psuedocode in: https://www.dropbox.com/sh/cp90q8nlk8od4cw/AADK4L3qOh-OJtFzdi_8Moaka?dl=0&preview=Project_1_The_Eight_Puzzle_CS_170_2022.pdf
#referred to https://www.geeksforgeeks.org/uniform-cost-search-dijkstra-for-large-graphs/
#other two algorithms with heuristic based on this algorithm
def UniformCostSearch(goal):
    #priority queue to store the solutionArrray and keep track of visited nodes
    SolutionPath = []
    visitedNodes = []
    expendedNodes = 0  

    # insert the starting index
    SolutionPath.append(start)

    # while the queue is not empty
    while (len(SolutionPath) > 0):
        child = list(SolutionPath)
        for x in child:
            if puzzle.CheckGoal(goal): #exit if same as goal
                return x
            # If child not in seen, Add the child to the queue
            if x.returnMatrix().list() not in visitedNodes:
                x.expended = expendedNodes #Update expended node variable
                solutionArrray.append(x)
        
 
    return SolutionPath

def ManhattanDistance(start, goal):
    #priority queue to store the solutionArrray and keep track of visited nodes
    SolutionPath = []
    SolutionPath.append(start)
    visitedNodes = []
    expendedNodes = 0  

    # insert the starting index
    SolutionPath.append(start)

    # while the queue is not empty
    while (len(SolutionPath) > 0):
        child = list(SolutionPath)
        for x in child:
            if x.puzzle.CheckGoal(goal): #exit if same as goal
                return x
            # If child not in seen, Add the child to the queue
            if x.returnMatrix().tolist() not in visitedNodes:
                x.cost = ManhattanDistanceCount(x.returnMatrix(), goal) + x.height #f(n) = g(n) + h(n)
                x.expended = expendedNodes #Update expended node variable
                solutionArrray.append(x)

    return SolutionPath

def MisplacedTile(start, goal):
    #priority queue to store the solutionArrray and keep track of visited nodes
    SolutionPath = []
    SolutionPath.append(start)
    visitedNodes = []
    expendedNodes = 0  

    # insert the starting index
    SolutionPath.append(start)

    # while the queue is not empty
    while (len(SolutionPath) > 0):
        child = list(SolutionPath)
        for x in child:
            if x.puzzle.CheckGoal(goal): #exit if same as goal
                return x
            # If child not in seen, Add the child to the queue
            if x.returnMatrix().tolist() not in visitedNodes:
                x.cost = MisplacedTileCount(x.returnMatrix(), goal) + x.height #f(n) = g(n) + h(n)
                x.expended = expendedNodes #Update expended node variable
                solutionArrray.append(x)

    return SolutionPath

#main console interaction part
print("")
print("Hi! Welcome to Mihir's Solver.") 
print("")
print("")
print("")
print("Enter 'sample' to use a sample puzzle, or 'own' to create your own.")
print("")
print("")
print("")
print("")
selection = input()    
if selection == 'sample':
    print("The sample puzzle was chosen.")
    print("")
    print("")
    print(f"Enter {1} for Depth 0, {2} for Depth 2, {3} for Depth 4, {4} for Depth 8, {5} for Depth 16, {7} for Depth 24 matrices")
    print("")
    print("")
    selection = input()
if selection == 1:
        print("Depth 0 GOAL state selected")
        print("")
        print("")
        puzzleChoice = puzzleGoatState0
elif selection == 2:
        print("Depth 2 GOAL state selected")
        print("")
        print("")
        puzzleChoice = puzzleGoatState2
elif selection == 3:
        print("Depth 4 GOAL state selected")
        print("")
        print("")
        puzzleChoice = puzzleGoatState4
elif selection == 4:
        print("Depth 8 GOAL State selected")
        print("")
        print("")
        puzzleChoice = puzzleGoatState8
elif selection == 5:
        print("Depth 16 GOAL state selected")
        print("")
        print("")
        puzzleChoice = puzzleGoatState16
elif selection == 6:
        print("Depth 20 GOAL state selected")
        print("")
        print("")
        puzzleChoice = puzzleGoatState20
elif selection == 7:
        print("Depth 24 GOAL state selected")
        print("")
        print("")
        puzzleChoice = puzzleGoatState24
        
else:
        print("Depth 16 selected")
        print("")
        print("")
        puzzleChoice = puzzleGoatState16
        
puzzle.printMatrix(puzzleChoice)
        
if input == 'own':
        print("\nYou selected to create your own puzzle. Enter your values and use a zero to represent blanks. Enter each row, use space between numbers")
        print("")
        print("")
        print("")
        print("")
        print("Enter the first row")
        print("")
        print("")
        firstRow = input()
        print("Enter the second row")
        print("")
        print("")
        secondRow = input()
        print("Enter the third row")
        print("")
        print("")
        thirdRow = input()

        #arrange as numpy as fixed size and same type of elements 
        puzzleChoice = numpy.array(
                    [[int(firstRow[0]),int(firstRow[2]),int(firstRow[4])],
                    [int(secondRow[0]),int(secondRow[2]),int(secondRow[4])],
                    [int(thirdRow[0]),int(thirdRow[2]),int(thirdRow[4])]])

start = puzzle(puzzleChoice)
    
print("")
print("")
print("What algorithm you want to use? Type '1' for Uniform Cost Search, '2' for A* with the Misplaced Tile heuristic, or '3' for A* with the Manhattan distance heuristic.")
input = input()
print("")
print("")
    
if input == 1:
        print("Uniform Cost Search selected")
        print("")
        print("")
        #time to run the algorithm
        startingPoint = time.time()
        solution = UniformCostSearch(puzzleGoalStateDefault)
        endingPoint = time.time()
        
        solutionArrray = []
        while(solution.parent != False):
            solutionArrray.append((solution.getMatrix(), solution.depth))
            solution = solution.parent

        print(f"Depth: {solutionArrray[1]} + Nodes Expended: {solutionArrray[2]}")
        print("")
        print("")
        print()    

if input == 2:
        print("Misplaced Tile heuristic selected")
        #time to run the algorithm
        startingPoint = time.time()
        solution = MisplacedTile(start, puzzleGoalStateDefault)
        endingPoint = time.time()
        
        #Store the solution solutionArrray to an array, then print it out with the corresponding height
        solutionArrray = []
        while(solution.parent != False):
            solutionArrray.append((solution.getMatrix(), solution.depth))
            solution = solution.parent

        print(f"Depth: {solutionArrray[1]} + Nodes Expended: {solutionArrray[2]}")
        print("")
        print("")
        print()     

if input == 3:
        print("Manhattan distance heuristic selected")
        #time to run the algorithm
        startingPoint = time.time()
        solution = ManhattanDistance(start, puzzleGoalStateDefault)
        endingPoint = time.time()
        
        solutionArrray = [] #solution goes here
        while(solution.parent == False):
            solutionArrray.append((solution.getMatrix(), solution.depth)) #solution + depth
            solution = solution.parent

        print(f"Depth: {solutionArrray[1]} + Nodes Expended: {solutionArrray[2]}")
        print("")
        print("")
        print()


#print f allows us to process the variables (source: GeeksforGeeks
# https://www.geeksforgeeks.org/formatted-string-literals-f-strings-python)
totalTime = endingPoint - startingPoint   
print(f"Time it takes for variables to process: {totalTime} seconds")
print("")
print("")
