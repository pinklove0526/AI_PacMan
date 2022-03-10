import random

from game import Agent
from game import Directions
from game import Actions
import search
import time


class GoWestAgent(Agent):
    def getAction(self, state):
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP


class RandomAgent(Agent):
    def getAction(self, state):
        actions = state.getLegalPacmanActions()
        random.shuffle(actions)
        return actions[0]


class SearchAgent(Agent):
    def registerInitialState(self, state):
        """
        This is the first time that the agent sees the layout of the game
        board. Here, we choose a path to the goal. In this phase, the agent
        should compute the path to the goal and store it in a local variable.
        All of the work is done in this method!

        state: a GameState object (pacman.py)
        """
        # TODO 11
        "No search function provided for SearchAgent"
        if self.searchFunction == None: raise Exception
        starttime = time.time()
        problem = self.searchType(state)  # Makes a new search problem
        self.actions = self.searchFunction(problem)  # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Path found with total cost of %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Search nodes expanded: %d' % problem._expanded)
    def getAction(self, state):
        """
        Returns the next action in the path chosen earlier (in
        registerInitialState).  Return Directions.STOP if there is no further
        action to take.

        state: a GameState object (pacman.py)
        """
        # TODO 12
        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP


class FoodSearchProblem:
    """
    A search problem associated with finding the a path that collects all of the
    food (dots) in a Pacman game.

    A search state in this problem is a tuple ( pacmanPosition, foodGrid ) where
      pacmanPosition: a tuple (x,y) of integers specifying Pacman's position
      foodGrid:       a Grid (see game.py) of either True or False, specifying remaining food
    """

    def __init__(self, startingGameState):
        self.start = (startingGameState.getPacmanPosition(),
                      startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.ghostPositions = [(int(x), int(y)) for (
            x, y) in startingGameState.getGhostPositions()]
        self.startingGameState = startingGameState
        self._expanded = 0  # DO NOT CHANGE
        self.heuristicInfo = {}  # A dictionary for the heuristic to store information

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        "Returns successor states, the actions they require, and a cost of 1."
        successors = []
        self._expanded += 1  # DO NOT CHANGE
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append((((nextx, nexty), nextFood), direction, 1))
        return successors

    def getCostOfActions(self, actions):
        """Returns the cost of a particular sequence of actions.  If those actions
        include an illegal move, return 999999"""
        x, y = self.getStartState()[0]
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost

class BFSFoodSearchAgent(SearchAgent):
    # TODO 13
    pass
    def __init__(self, fn='breadthFirstSearch', prob='FoodSearchProblem', heuristic='nullHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)


class DFSFoodSearchAgent(SearchAgent):
    # TODO 14
    pass
    def __init__(self, fn='depthFirstSearch', prob='FoodSearchProblem', heuristic='nullHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)


class UCSFoodSearchAgent(SearchAgent):
    # TODO 15
    pass
    def __init__(self, fn='uniformCostSearch', prob='FoodSearchProblem', heuristic='nullHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)


class AStarFoodSearchAgent(SearchAgent):
    # TODO 16
    pass

    def __init__(self, fn='aStarSearch', prob='FoodSearchProblem', heuristic='nullHeuristic'):
        SearchAgent.__init__(self, fn, prob, heuristic)

    def foodHeuristic(state, problem):
        # TODO
        position, foodGrid = state
        return len(foodGrid.asList())

        foodToEat = foodGrid.asList()
        totalCost = 0
        curPoint = position
        while foodToEat:
            heuristic_cost, food = \
                min([(util.manhattanDistance(curPoint, food), food) for food in foodToEat])
            foodToEat.remove(food)
            curPoint = food
            totalCost += heuristic_cost

        return totalCost
        pass

