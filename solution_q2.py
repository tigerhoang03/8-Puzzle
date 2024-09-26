#changing the goal state will only need you to really change the successor function and goal state of course
import heapq  #For the priority queue
from collections import deque
import math  #For square root calculation

#Andrew Hoang ath5428
#Note: I used geeksforgeeks to help with my bfs dfs algo implementation as it has been a while

"""---------------------------------------------------------------------------------------------------------------------------"""

#Data type for the search tree
class Node:
    def __init__(self, state, parent=None, action=None, cost=0, heuristic=0): 
        self.state = state 
        self.parent = parent
        self.action = action
        self.cost = cost  #Cost to reach this node (g)
        self.heuristic = heuristic  #Heuristic estimate to goal (h)
        self.totalCost = cost + heuristic  #Total cost (f = g + h)

    #Comparator for the priority queue
    def __lt__(self, other):
        return self.totalCost < other.totalCost

"""Below is NOT part of the class"""

def readFile(file):
    with open(file, 'r') as file:
        #Reads the one input line from input.txt
        line = file.readline().strip()
        
    elements = line.split(',')
    initialState = []

    for elem in elements:        
        if elem == '_':
            initialState.append(0)            
        else:
            initialState.append(int(elem))
                
    return initialState  

#Note: these coordinates are zero-based  
def getCoord(startState):
    #Use this to figure out the coordinates of the blank square as from there we can figure out what can move
    row = startState.index(0) // 3  #row index calc
    column = startState.index(0) % 3  #Will return the 'remainder' or the column position
    return row, column
        
#We now need to generate the valid moves from the curr state and return them
#Each element in the moves array would represent one new state since at each step we can only move one direction
def getSuccessors(state):
    moves = []  #Will store the possible states and the moves that will get you there from the curr state
    row, col = getCoord(state)

    #Helper function to perform the swap and return new state and action
    def swap(newRow, newCol, direction):
        newState = state[:]  #Create a copy of the curr state
        oldIdx = row * 3 + col  #curr state blank index
        newIdx = newRow * 3 + newCol  #New state blank index

        #The tile being moved is the one at newIdx
        tileMoved = newState[newIdx]
        newState[oldIdx], newState[newIdx] = newState[newIdx], newState[oldIdx]

        action = f"{tileMoved}{direction}"

        return newState, action

    #Generate possible moves with corresponding actions
    if row < 2:  
        moves.append(swap(row + 1, col, 'U'))  #up
    if row > 0:  
        moves.append(swap(row - 1, col, 'D'))  #down
    if col < 2:  
        moves.append(swap(row, col + 1, 'L'))  #left
    if col > 0:  
        moves.append(swap(row, col - 1, 'R'))  #right
    return moves 

#Goal test: sum of top row should be 11
def isGoalState(state):
    return sum(state[:3]) == 11 

"""BELOW ARE THE ALGOS."""

#Reconstruct the path and actions
def reconstructPath(goalNode):
    actions = []
    currNode = goalNode
    
    while currNode.parent is not None:
        actions.append(currNode.action)
        currNode = currNode.parent

    actions.reverse()
    return actions

#Manhattan distance heuristic
def manhattanDistance(state, goalState):
    distance = 0
    for i in range(1, 9):  #Ignore 0 (blank space)
        currentIdx = state.index(i)
        goalIdx = goalState.index(i)
        
        currentRow, currentCol = currentIdx // 3, currentIdx % 3
        goalRow, goalCol = goalIdx // 3, goalIdx % 3
        
        distance += abs(currentRow - goalRow) + abs(currentCol - goalCol)
    return distance

#Straight-Line Distance (Euclidean distance) heuristic
def straightLineDistance(state, goalState):
    distance = 0
    for i in range(1, 9):  #Ignore 0 (blank space)
        currentIdx = state.index(i)
        goalIdx = goalState.index(i)
        
        currentRow, currentCol = currentIdx // 3, currentIdx % 3
        goalRow, goalCol = goalIdx // 3, goalIdx % 3
        
        #Calculate the Euclidean distance between the current position and goal position
        distance += math.sqrt((currentRow - goalRow) ** 2 + (currentCol - goalCol) ** 2)
    
    return distance

#DFS implementation
def dfs(initialState):
    initialNode = Node(initialState)
    stack = [initialNode]
    visited = set()
    expanded = 0   

    while stack:
        currNode = stack.pop()  
        currState = currNode.state

        if isGoalState(currState):  #Check for completion using new goal test
            return currNode, expanded 
        visited.add(tuple(currState))
        for successorState, action in getSuccessors(currState):
            if tuple(successorState) not in visited:
                successorNode = Node(successorState, parent=currNode, action=action)
                stack.append(successorNode)
        expanded += 1   

    return None, expanded   

#BFS implementation
def bfs(initialState):
    initialNode = Node(initialState)
    queue = deque([initialNode])  #Use deque for optimized FIFO queue
    visited = set()
    expanded = 0   

    while queue:
        currNode = queue.popleft() 
        currState = currNode.state
        
        if isGoalState(currState):
            return currNode, expanded  #Return the goal node and number of expansions
        
        visited.add(tuple(currState))
        for successorState, action in getSuccessors(currState):
            if tuple(successorState) not in visited:
                successorNode = Node(successorState, parent=currNode, action=action)
                queue.append(successorNode)
        expanded += 1   

    return None, expanded   

#UCS implementation
def uniformCostSearch(initialState):
    initialNode = Node(initialState, cost=0)
    priorityQueue = []  #Priority queue based on cost
    heapq.heappush(priorityQueue, initialNode)
    
    visited = set()
    expanded = 0   

    while priorityQueue:
        currNode = heapq.heappop(priorityQueue)  #Pop node with the lowest cost
        currState = currNode.state

        if isGoalState(currState):
            return currNode, expanded 

        visited.add(tuple(currState))
        for successorState, action in getSuccessors(currState):
            if tuple(successorState) not in visited:
                newCost = currNode.cost + 1  #Assuming cost for every move is 1
                successorNode = Node(successorState, parent=currNode, action=action, cost=newCost)
                heapq.heappush(priorityQueue, successorNode)
        expanded += 1   

    return None, expanded   

#A* Search implementation with dynamic heuristic
def aStarSearch(initialState, heuristicFunc=manhattanDistance):  #Default to manhattanDistance if no heuristic is provided
    initialNode = Node(initialState, cost=0, heuristic=heuristicFunc(initialState, [0, 1, 2, 3, 4, 5, 6, 7, 8]))
    priorityQueue = []  #Priority queue use total cost (f = g + h)
    heapq.heappush(priorityQueue, initialNode)
    
    visited = set()
    expanded = 0  

    while priorityQueue:
        currNode = heapq.heappop(priorityQueue)  #Pop node with the lowest f = g + h
        currState = currNode.state

        if isGoalState(currState):
            return currNode, expanded  
        visited.add(tuple(currState))

        for successorState, action in getSuccessors(currState):
            if tuple(successorState) not in visited:
                gCost = currNode.cost + 1  #Uniform cost for each move is 1
                hCost = heuristicFunc(successorState, [0, 1, 2, 3, 4, 5, 6, 7, 8])
                successorNode = Node(successorState, parent=currNode, action=action, cost=gCost, heuristic=hCost)
                heapq.heappush(priorityQueue, successorNode)

        expanded += 1  

    return None, expanded  

if __name__ == "__main__":
    startState = readFile("input.txt")
    
    print("The actual returned path by DFS:")
    goalNode, expansions = dfs(startState)
    if goalNode:
        actions = reconstructPath(goalNode)
        print(",".join(actions))
        #print(f"Nodes expanded: {expansions}")
    else:
        print("No solution found.")
        
    print("\nThe actual returned path by BFS:")
    goalNode, expansions = bfs(startState)
    if goalNode:
        actions = reconstructPath(goalNode)
        print(",".join(actions))  
        #print(f"Nodes expanded: {expansions}")
    else:
        print("No solution found.")
    
    print("\nThe actual returned path by UCS:")
    goalNode, expansions = uniformCostSearch(startState)
    if goalNode:
        actions = reconstructPath(goalNode)
        print(",".join(actions))  
        #print(f"Nodes expanded: {expansions}")
    else:
        print("No solution found.")

    #A* search execution with Manhattan Distance
    print("\nThe actual returned path by A* Search with Manhattan Distance:")
    goalNode, expansions = aStarSearch(startState, manhattanDistance)
    if goalNode:
        actions = reconstructPath(goalNode)
        print(",".join(actions))  
        #print(f"Nodes expanded: {expansions}")
    else:
        print("No solution found.")
    
    #A* search execution with Straight-Line (Euclidean) Distance
    print("\nThe actual returned path by A* Search with Straight-Line Distance:")
    goalNode, expansions = aStarSearch(startState, straightLineDistance)
    if goalNode:
        actions = reconstructPath(goalNode)
        print(",".join(actions))  
        #print(f"Nodes expanded: {expansions}")
    else:
        print("No solution found.")
