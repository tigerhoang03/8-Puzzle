#Andrew Hoang ath5428
#plan: read the input.txt file and turn it into a 2d list
#similarly the goal state can be the same too

#according to the textbook goal formulation is the first step so our goalstate is:
# goalState = (0, 1, 2, 3, 4, 5, 6, 7, 8)

#next we need to define the problem
#inital state -> taken care of by reading the file in and making an inital state representation
#actions -> taken care of by possMoves function which provides all possible moves 
#successor function -> apart of the possMoves fn.
"""---------------------------------------------------------------------------------------------------------------------------"""

#data type for the search tree
class Node:
    def __init__(self, startState, parent=None): 
        self.startState = startState 
        self.parent= parent

"""below is NOT apart of the class"""

def readFile(file):
    with open(file, 'r') as file:
        #reads the one input line from input.txt
        line = file.readline().strip()
        
    elements = line.split(',')
    initalState = []

    for elem in elements:        
        if elem == '_':
            initalState.append(0)            
        else:
            initalState.append(int(elem))
            
    return initalState #jsut to ensure immutability

#Note: these coordinates are zero based  
def coord(startState):#use this to figure out the coordinates of the blank square as from there we can figure out what can move
    row = startState.index(0) // 3 #figures out the index and does floor division (zero based indexing clutches yet)
    column = startState.index(0) % 3 #will return the 'remainder' or the column position
    return row, column
    
#we now need to generate the valid moves from the current state and return them
#each element in the moves array would represent one new state since at each step we can only move one direction
def moves():
    moves = []
    

goalState = [0, 1, 2, 3, 4, 5, 6, 7, 8]
if __name__ == "__main__":
    startState = readFile("input.txt")
    print(coord(startState))
    