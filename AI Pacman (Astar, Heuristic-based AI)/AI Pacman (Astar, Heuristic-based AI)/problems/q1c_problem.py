import logging
import time
from typing import Tuple

import util
from game import Actions, Agent, Directions
from logs.search_logger import log_function
from pacman import GameState


class q1c_problem:
    """
    A search problem associated with finding a path that collects all of the
    food (dots) in a Pacman game.
    Some useful data has been included here for you
    """
    def __str__(self):
        return str(self.__class__.__module__)

    def __init__(self, gameState: GameState):
        """
        Stores the start and goal.

        gameState: A GameState object (pacman.py)
        costFn: A function from a search state (tuple) to a non-negative number
        goal: A position in the gameState
        """
        self.startingGameState: GameState = gameState

    @log_function
    def getStartState(self):
        "*** YOUR CODE HERE ***"
        self.goodFood = self.find_goodFood(self.startingGameState.getPacmanPosition(), self.startingGameState.getFood().asList())
        return (self.startingGameState.getPacmanPosition(), tuple(self.goodFood))

    @log_function
    def isGoalState(self, state):
        "*** YOUR CODE HERE ***"
        return len(state[1]) == 0
            

    @log_function
    def getSuccessors(self, state):
        """
        Returns successor states, the actions they require, and a cost of 1.

         As noted in search.py:
             For a given state, this should return a list of triples,
         (successor, action, stepCost), where 'successor' is a
         successor to the current state, 'action' is the action
         required to get there, and 'stepCost' is the incremental
         cost of expanding to that successor
        """
        "*** YOUR CODE HERE ***"
        
        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state[0]
            dx, dy = Actions.directionToVector(action)
            next_x, next_y = int(x + dx), int(y + dy)
            if not self.startingGameState.hasWall(next_x, next_y):
                newFood = list(state[1])
                if (next_x, next_y) in newFood:
                    newFood.remove((next_x, next_y))
                successors.append((((next_x, next_y), tuple(newFood)), action, 1))

        return successors
    
    def find_goodFood(self, state, food):
        # Run BFS to find which food is reachable from Pacman position, return the food that is reachable
        start = state
        queue = util.Queue()
        visited = set()
        goodFood = []

        queue.push(start)

        while not queue.isEmpty():
            current = queue.pop()
            if current in visited:
                continue
            if current in food:
                goodFood.append(current)
                
            visited.add(current)
            x, y = current
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                next_x, next_y = x + dx, y + dy
                if (next_x, next_y) not in visited and not self.startingGameState.hasWall(next_x, next_y):
                    queue.push((next_x, next_y))

        return goodFood