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


class BFSFoodSearchAgent(SearchAgent):
    # TODO 13
    pass


class DFSFoodSearchAgent(SearchAgent):
    # TODO 14
    pass


class UCSFoodSearchAgent(SearchAgent):
    # TODO 15
    pass


class AStarFoodSearchAgent(SearchAgent):
    # TODO 16
    pass

    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, foodHeuristic)
        self.searchType = FoodSearchProblem


    def foodHeuristic(state, problem):
        """Encourage Pacman to eat all the pellets as fast as possible."""
        position, foodGrid = state

        heuristic = 0
        foodList = foodGrid.asList()

        # calculate the distance from current node to food-containing nodes
        if len(foodList) > 0:
            closestPoint = findClosestPoint(position, foodList)
            farthestPoint = findFarthestPoint(position, foodList)

            closestPointIndex = closestPoint[0]
            farthestPointIndex = farthestPoint[0]

            currentNode = problem.startingGameState
            closestFoodNode = foodList[closestPointIndex]
            farthestFoodNode = foodList[farthestPointIndex]

            # distance between current location and closest manhattan node
            currentToClosest = mazeDistance(position, closestFoodNode, currentNode)

            # distance between closest manhattan node and farthest manhattan node
            closestToFarthest = mazeDistance(closestFoodNode, farthestFoodNode, currentNode)

            heuristic = currentToClosest + closestToFarthest

        return heuristic

