from collections import deque
import heapq

class State:
    def __init__(self, missionaries, cannibals, boat):
        self.missionaries = missionaries
        self.cannibals = cannibals
        self.boat = boat

    def isValid(self):
        if (self.missionaries < 0 or self.cannibals < 0 or self.missionaries > 3 or self.cannibals > 3):
            return False
        if (self.cannibals > self.missionaries and self.missionaries > 0 ):
            return False
        if (3 - self.cannibals > 3 - self.missionaries and 3 - self.missionaries > 0):
            return False
        return True

    def checkGoal(self):
        return self.missionaries == 0 and self.cannibals == 0 and self.boat == 1

def getValidActions(state):
    actions = []
    for i in range(3):
        for j in range(3):
            if 0 < i + j <= 2:
                actions.append((i, j))
    return actions

def implementAction(state, action):
    missionaries, cannibals = action
    if state.boat == 0:
        newState = State(
            state.missionaries - missionaries,
            state.cannibals - cannibals,
            1,
        )
    else:
        newState = State(
            state.missionaries + missionaries,
            state.cannibals + cannibals,
            0,
        )
    return newState

def bfs():
    startState = State(3, 3, 0)
    if startState.checkGoal():
        return [startState]

    visited = set()
    queue = deque([([startState], [])])

    while queue:
        path, actions = queue.popleft()
        currentState = path[-1]

        for action in getValidActions(currentState):
            nextState = implementAction(currentState, action)
            if nextState not in visited and nextState.isValid():
                newPath = list(path)
                newPath.append(nextState)
                newActions = list(actions)
                newActions.append(action)

                if nextState.checkGoal():
                    return newPath, newActions

                visited.add(nextState)
                queue.append((newPath, newActions))

    return None, None

pathBFS, actionsBFS = bfs()

if pathBFS is not None:
    print("BFS Solution:")
    for i, state in enumerate(pathBFS):
        print(f"Step {i}: {state.missionaries}M-{state.cannibals}C-{state.boat}B")
    print("Actions taken to reach the goal state")
    for action in actionsBFS:
        print(f"{action[0]}-{action[1]}")
else:
    print("BFS: No solution found.")

def dfsLimit(state, depth, visited, path, actions):
    if depth == 0:
        return None, None

    visited.add(state)
    path.append(state)

    if state.checkGoal():
        return path, actions

    for action in getValidActions(state):
        nextState = implementAction(state, action)

        if nextState not in visited and nextState.isValid():
            newActions = list(actions)
            newActions.append(action)
            resultPath, resultActions = dfsLimit(nextState, depth - 1, visited, path, newActions)

            if resultPath is not None:
                return resultPath, resultActions

    path.pop()
    return None, None

def dfs():
    startState = State(3, 3, 0)

    for depth in range(1, 100):  # You can adjust the maximum depth as needed
        visited = set()
        path, actions = dfsLimit(startState, depth, visited, [], [])

        if path is not None:
            return path, actions

    return None, None

pathDFS, actionsDFS = dfs()

if pathDFS is not None:
    print("DFS Solution:")
    for i, state in enumerate(pathDFS):
        print(f"Step {i}: {state.missionaries}M-{state.cannibals}C-{state.boat}B")
    print("Actions taken to reach the goal state:")
    for action in actionsDFS:
        print(f"{action[0]}-{action[1]}")
else:
    print("DFS: No solution found.")

def heuristic(state):
    return state.missionaries + state.cannibals

def greedyBestFirstSearch():
    startState = State(3, 3, 0)
    if startState.checkGoal():
        return [startState]

    openNodes = [(heuristic(startState), [startState])]
    closedSet = set()

    while openNodes:
        openNodes.sort(key=lambda x: x[0])
        _, path = openNodes.pop(0)
        currentState = path[-1]
        closedSet.add(currentState)

        if currentState.checkGoal():
            return path

        for action in getValidActions(currentState):
            nextState = implementAction(currentState, action)

            if nextState not in closedSet and nextState.isValid():
                cost = len(path) + heuristic(nextState)
                newPath = list(path)
                newPath.append(nextState)
                openNodes.append((cost, newPath))

    return None

greedyPath = greedyBestFirstSearch()

if greedyPath is not None:
    print("Greedy Best-First Search Solution:")
    for i, state in enumerate(greedyPath):
        print(f"Step {i}: {state.missionaries}M-{state.cannibals}C-{state.boat}B")
else:
    print("Greedy Best-First Search: No solution found.")


def aStarSearch():
    startState = State(3, 3, 0)
    if startState.checkGoal():
        return [startState]

    openNodes = [(heuristic(startState), [startState])]
    closedSet = set()

    while openNodes:
        openNodes.sort(key=lambda x: x[0])
        _, path = openNodes.pop(0)
        currentState = path[-1]
        closedSet.add(currentState)

        if currentState.checkGoal():
            return path

        for action in getValidActions(currentState):
            nextState = implementAction(currentState, action)

            if nextState not in closedSet and nextState.isValid():
                cost = len(path) + heuristic(nextState)
                newPath = list(path)
                newPath.append(nextState)
                openNodes.append((cost, newPath))

    return None

aStarPath = aStarSearch()

if aStarPath is not None:
    print("The A* Algorithm gives the following solution")
    for i, state in enumerate(aStarPath):
        print(f"Step {i}: {state.missionaries}M-{state.cannibals}C-{state.boat}B")
else:
    print("A* Search: No solution found.")
