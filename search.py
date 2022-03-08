"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from game import Directions
import util
n = Directions.NORTH
s = Directions.SOUTH
e = Directions.EAST
w = Directions.WEST


def depthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 17
    """Search the deepest nodes in the search tree first."""

    # states to be explored (LIFO). holds nodes in form (state, action)
    frontier = util.Stack()
    # previously explored states (for path checking), holds states
    explored = []
    # define start node
    startState = problem.getStartState()
    startNode = (startState, [])

    frontier.push(startNode)

    while not frontier.isEmpty():
        # begin exploring last (most-recently-pushed) node on frontier
        currentState, actions = frontier.pop()

        if currentState not in explored:
            # mark current node as explored
            explored.append(currentState)

            if problem.isGoalState(currentState):
                return actions
            else:
                # get list of possible successor nodes in
                # form (successor, action, stepCost)
                successors = problem.getSuccessors(currentState)

                # push each successor to frontier
                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newNode = (succState, newAction)
                    frontier.push(newNode)

    return actions


def breadthFirstSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 18
    frontier = util.Queue()

    # previously expanded states (for cycle checking), holds states
    explored = []

    startState = problem.getStartState()
    startNode = (startState, [], 0)  # (state, action, cost)

    frontier.push(startNode)

    while not frontier.isEmpty():
        # begin exploring first (earliest-pushed) node on frontier
        currentState, actions, currentCost = frontier.pop()

        if currentState not in explored:
            # put popped node state into explored list
            explored.append(currentState)

            if problem.isGoalState(currentState):
                return actions
            else:
                # list of (successor, action, stepCost)
                successors = problem.getSuccessors(currentState)

                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newCost = currentCost + succCost
                    newNode = (succState, newAction, newCost)

                    frontier.push(newNode)

    return actions


def uniformCostSearch(problem):
    '''
    return a path to the goal
    '''
    # TODO 19
    frontier = util.PriorityQueue()

    # previously expanded states (for cycle checking), holds state:cost
    exploredNodes = {}

    startState = problem.getStartState()
    startNode = (startState, [], 0)  # (state, action, cost)

    frontier.push(startNode, 0)

    while not frontier.isEmpty():
        # begin exploring first (lowest-cost) node on frontier
        currentState, actions, currentCost = frontier.pop()

        if (currentState not in exploredNodes) or (currentCost < exploredNodes[currentState]):
            # put popped node's state into explored list
            exploredNodes[currentState] = currentCost

            if problem.isGoalState(currentState):
                return actions
            else:
                # list of (successor, action, stepCost)
                successors = problem.getSuccessors(currentState)

                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newCost = currentCost + succCost
                    newNode = (succState, newAction, newCost)

                    frontier.update(newNode, newCost)

    return actions

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def singleFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of single food search
    """
    # TODO 20
    pass
    x= state
    y= problem.getStartState()



def multiFoodSearchHeuristic(state, problem=None):
    """
    A heuristic function for the problem of multi-food search
    """
    # TODO 21
    pass
def manhattanHeuristic(position, problem, info={}):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])

def euclideanHeuristic(position, problem, info={}):
    "The Euclidean distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return ( (xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2 ) ** 0.5

def aStarSearch(problem, heuristic=nullHeuristic):
    '''
    return a path to the goal
    '''
    # TODO 22
    startState = problem.getStartState()
    # a * using priority queue so as to prioriize the successors with least
    # heuristic cost
    fringe = util.PriorityQueue()
    visited = []

    # the fringe apart from the state , action, cost also has combined cost 0 here
    # of which is we want the least combined cost first
    fringe.push((startState, [], 0), 0)

    # keep popping till no more nodes in the fringe
    while not fringe.isEmpty():
        currentState, actions, costs = fringe.pop()
        # curcial as this prevents expanding the same node twice
        if not currentState in visited:
            # update visited status
            visited.append(currentState)
            # if this goal state return the actions to reach it
            if problem.isGoalState(currentState):
                return actions
            # push all successors not in visited
            for state, action, cost in problem.getSuccessors(currentState):
                if not state in visited:
                    # update cost to reflect combined path and heuristic cost
                    # and prioritize the least for popping as the priority queue
                    # is implemeneted using heapq which pops smallest element
                    # first and pushes such to maintain this order
                    heuristicCost = costs + cost + heuristic(state, problem)
                    fringe.push((state, actions + [action], costs + cost), heuristicCost)
    util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
