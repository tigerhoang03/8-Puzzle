# solution.py

def readInitialState(filename):
    """Reads the initial state from the input file."""
    with open(filename, 'r') as file:
        line = file.readline().strip()
    elements = line.split(',')
    state = []
    for elem in elements:
        if elem == '_':
            state.append(0)  # Represent the empty tile with 0
        else:
            state.append(int(elem))
    return tuple(state)  # Use a tuple for immutability

def getPossibleMoves(state):
    """Generates all possible moves from the current state, along with the actions."""
    moves = []
    index = state.index(0)  # Find the empty tile (represented by 0)
    row, col = divmod(index, 3)
    # Prioritize directions: Up, Left, Right, Down
    directions = [
        (-1, 0),  # Up
        (0, -1),  # Left
        (0, 1),   # Right
        (1, 0)    # Down
    ]
    # Mapping for the direction the tile moves
    directionMap = {
        (1, 0): 'U',   # Tile moves Up
        (-1, 0): 'D',  # Tile moves Down
        (0, 1): 'L',   # Tile moves Left
        (0, -1): 'R',  # Tile moves Right
    }

    for deltaRow, deltaCol in directions:
        newRow, newCol = row + deltaRow, col + deltaCol
        if 0 <= newRow < 3 and 0 <= newCol < 3:
            newIndex = newRow * 3 + newCol
            newState = list(state)
            # Swap the empty tile with the adjacent tile
            newState[index], newState[newIndex] = newState[newIndex], newState[index]
            # The tile that moves is newState[index] after the swap
            tile = newState[index]
            # Determine the movement of the tile
            tileDeltaRow, tileDeltaCol = -deltaRow, -deltaCol  # Invert the direction
            direction = directionMap[(tileDeltaRow, tileDeltaCol)]
            action = f"{tile}{direction}"
            moves.append((tuple(newState), action))
    return moves

def reconstructPath(parent, state):
    """Reconstructs the path and actions from the initial state to the goal state."""
    path = []
    actions = []
    while state in parent:
        prevState, action = parent[state]
        path.append(state)
        actions.append(action)
        state = prevState
    path.append(state)  # Add the initial state
    path.reverse()
    actions.reverse()
    return path, actions

def dfs(initialState):
    """Performs Depth-First Search to find the solution path."""
    GOAL_STATE = (0, 1, 2, 3, 4, 5, 6, 7, 8)
    stack = [initialState]
    visited = set()
    parent = {}  # To reconstruct the path and actions

    while stack:
        currentState = stack.pop()
        if currentState in visited:
            continue
        visited.add(currentState)

        if currentState == GOAL_STATE:
            return reconstructPath(parent, currentState)

        # Reverse the moves to ensure correct order
        for neighbor, action in reversed(getPossibleMoves(currentState)):
            if neighbor not in visited:
                parent[neighbor] = (currentState, action)
                stack.append(neighbor)
    return None, None  # No solution found

if __name__ == "__main__":
    """Main block to execute the 8-puzzle solver."""
    initialState = readInitialState('input.txt')
    path, actions = dfs(initialState)
    if path:
        print(f"Initial input is: {','.join('_' if x == 0 else str(x) for x in initialState)}")
        print("Actual returned path:", ','.join(actions))
    else:
        print("No solution found.")
