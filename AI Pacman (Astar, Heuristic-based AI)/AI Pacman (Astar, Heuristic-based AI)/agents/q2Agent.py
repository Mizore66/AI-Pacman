import logging
import random

import util
from game import Actions, Agent, Directions
from logs.search_logger import log_function
from pacman import GameState
from util import manhattanDistance


def scoreEvaluationFunction(currentGameState):
    """
      This default evaluation function just returns the score of the state.
      The score is the same one displayed in the Pacman GUI.

      This evaluation function is meant for use with adversarial search agents
      (not reflex agents).
    """
    return currentGameState.getScore()

class Q2_Agent(Agent):

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

    @log_function
    def getAction(self, gameState: GameState):
        """
            Returns the minimax action from the current gameState using self.depth
            and self.evaluationFunction.

            Here are some method calls that might be useful when implementing minimax.

            gameState.getLegalActions(agentIndex):
            Returns a list of legal actions for an agent
            agentIndex=0 means Pacman, ghosts are >= 1

            gameState.generateSuccessor(agentIndex, action):
            Returns the successor game state after an agent takes an action

            gameState.getNumAgents():
            Returns the total number of agents in the game
        """
        logger = logging.getLogger('root')
        logger.info('MinimaxAgent')
        "*** YOUR CODE HERE ***"
        self.deadEndAndCornergoals = self.getDeadEnds(gameState.getFood().asList(), gameState) + self.getCorners(gameState.getFood().asList(), gameState)


        _, action = self.alpha_beta_pruning(gameState, self.depth, float('-inf'), float('inf'), 0)

        # pacman_position = gameState.getPacmanPosition()
        # if pacman_position in self.deadEndAndCornergoals:
        #     self.deadEndAndCornergoals.remove(pacman_position)


        return action
    def alpha_beta_pruning(self, state, depth, alpha, beta, agent):
        value, move = self.max_value(state, state, depth, alpha, beta, agent)
        return value, move
    
    def max_value(self, state, oldState, depth, alpha, beta, agent):
        if state.isWin() or state.isLose() or depth == 0:
            score = self.heuristicEvaluation(state, oldState)
            return score, None
        value = float('-inf')
        action = None
        for a in state.getLegalActions(agent):
            successor = state.generateSuccessor(agent, a)

            # successor_position = successor.getPacmanPosition()

            # if successor_position in self.deadEndAndCornergoals:
            #     return 999999999, a

            v, _ = self.min_value(successor, state, depth, alpha, beta, agent + 1)
            if v > value:
                value = v
                action = a
                alpha = max(alpha, value)
            
            if alpha >= beta:
                return value, action
            
        return value, action
    
    def min_value(self, state, oldState, depth, alpha, beta, agent):
        if state.isWin() or state.isLose() or depth == 0:
            score = self.heuristicEvaluation(state, oldState)
            return score, None
        value = float('inf')
        action = None
        for a in state.getLegalActions(agent):
            successor = state.generateSuccessor(agent, a)

            if agent == state.getNumAgents() - 1:
                v, _ = self.max_value(successor, state, depth - 1, alpha, beta, 0)
            else:
                v, _ = self.min_value(successor, state, depth, alpha, beta, agent + 1)
                
            if v < value:
                value = v
                action = a
                beta = min(beta, value)
            if beta <= alpha:
                return value, action
        return value, action
    
        
    
    def heuristicEvaluation(self, currentGameState: GameState, oldGameState: GameState):
        if currentGameState.isLose():      
            return -999999999 + scoreEvaluationFunction(currentGameState)
        
        scaredGhostHeuristic = 0

        for ghostState in currentGameState.getGhostStates():
            x, y = currentGameState.getPacmanPosition()

            if ghostState.scaredTimer >= 10:
                scaredGhostDistance, _ = self.mazeDistance((x, y), ghostState.getPosition(), currentGameState)
                scaredGhostHeuristic += 50000*scaredGhostDistance**-1 if scaredGhostDistance != 0 else 999999999 
            ghost_x, ghost_y = ghostState.getPosition()

        current_position = currentGameState.getPacmanPosition()
        closest_food_distance = self.mazeDistance(current_position, currentGameState.getFood().asList(), currentGameState)[0] + 1

        if not currentGameState.getFood().asList():
            return 999999999 + scoreEvaluationFunction(currentGameState)
        
        if self.ghostNearby(currentGameState):
            return -999999999 + scoreEvaluationFunction(currentGameState)

        #Check if there is food 1 space around pacman
        
        #Check if there is food 1 space around pacman
        # possible_neighbours = [(current_position[0] + 1, current_position[1]), (current_position[0] - 1, current_position[1]), (current_position[0], current_position[1] + 1), (current_position[0], current_position[1] - 1)]
        # for neighbour in possible_neighbours:
        #     if neighbour in self.deadEndAndCornergoals:
        #         return 99999 + scoreEvaluationFunction(currentGameState)

        # tsp_distance = self.greedy_travelling_salesman(current_position, currentGameState.getFood().asList(), currentGameState) + 1

        # tsp_heuristic = 35*tsp_distance**-1

        closest_capsule_distance = self.mazeDistance(current_position, currentGameState.getCapsules(), currentGameState)[0] + 1

        food_heuristic = 20*closest_food_distance**-1
        food_amount_heuristic = -100*len(currentGameState.getFood().asList())
        capsule_amount_heuristic = -100*len(currentGameState.getCapsules())
        capsule_heuristic = 50*closest_capsule_distance**-1

        

        return food_heuristic + capsule_heuristic + food_amount_heuristic + 200*scoreEvaluationFunction(currentGameState) + capsule_amount_heuristic + scaredGhostHeuristic# + tsp_heuristic

    def ghostNearby(self, succesor):
        x, y = succesor.getPacmanPosition()
        for ghost in succesor.getGhostStates():
            ghost_pos = ghost.getPosition()
            if manhattanDistance((x, y), ghost_pos) <= 1 and ghost.scaredTimer == 0:
                return True
        return False


    def mazeDistance(self, position, foods, gamestate):
        """
        Returns the maze distance between any two points, foods is a list of points, using BFS
        """
        if not foods:
            return 0, position
        fringe = util.Queue()
        fringe.push((position, 0))
        visited = set([position])

        while not fringe.isEmpty():
            pos, dist = fringe.pop()

            if pos in foods:
                return dist,pos

            x, y = pos
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                next_x, next_y = x + dx, y + dy
                if gamestate.hasWall(next_x, next_y) or (next_x, next_y) in visited:
                    continue
                visited.add((next_x, next_y))
                fringe.push(((next_x, next_y), dist + 1))

        return float('inf'), (-1, -1)
    
    def minManhattanDistance(self, position, goals):
        return min([manhattanDistance(position, goal) for goal in goals]) if goals else 0
    
    def getWalls(self, pos, gameState):
        #Get the count of walls around the position
        x, y = pos
        walls = 0
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            next_x, next_y = x + dx, y + dy
            if gameState.hasWall(next_x, next_y):
                walls += 1

        return walls
    
    def isCorner(self, pos, gameState):
        x, y = pos
        for dx, dy in [(-1, -1), (1, -1), (-1, 1), (1, 1)]:
            x_corner, y_corner = x + dx, y + dy
            if gameState.hasWall(x_corner, y) and gameState.hasWall(x, y_corner):
                return True
        return False
    
    def getDeadEnds(self, food, gameState):
        deadEnds = []
        for x, y in food:
            if self.getWalls((x, y), gameState) == 3:
                deadEnds.append((x, y))
        return deadEnds
    
    def getCorners(self, food, gameState):
        corners = []
        for x, y in food:
            if self.isCorner((x, y), gameState):
                corners.append((x, y))
        return corners
    
    def greedy_travelling_salesman(self, pos, foods, gameState):
        if not foods:
            return 0, []
        visited = set()
        visited.add(pos)
        total_distance = 0
        while foods:
            distance, closest_food = self.mazeDistance(pos, foods, gameState)
            total_distance += distance
            pos = closest_food
            foods.remove(closest_food)

        return total_distance