#---------------------#
# DO NOT MODIFY BEGIN #
#---------------------#

import logging

import util
from problems.q1a_problem import q1a_problem

def q1a_solver(problem: q1a_problem):
    astarData = astar_initialise(problem)
    num_expansions = 0
    terminate = False
    while not terminate:
        num_expansions += 1
        terminate, result = astar_loop_body(problem, astarData)
    print(f'Number of node expansions: {num_expansions}')
    return result

#-------------------#
# DO NOT MODIFY END #
#-------------------#

from game import Actions

class AStarData:
    def __init__(self):
        self.frontier = util.PriorityQueue()
        self.goal = None
        self.explored = {}
        self.paths = {}

def astar_initialise(problem: q1a_problem):
    astarData = AStarData()
    start_state = problem.getStartState()
    astarData.goal = problem.startingGameState.getFood().asList()[0]

    root_h = astar_heuristic(start_state, astarData.goal)
    astarData.frontier.push((0, start_state), root_h)
    astarData.paths[start_state] = []

    return astarData

def astar_loop_body(problem: q1a_problem, astarData: AStarData):
    if not astarData.frontier.heap:
        return True, []
    
    curr_g, current = astarData.frontier.pop()

    if problem.isGoalState(current):
        path = []
        while current != problem.getStartState():
            path = [astarData.paths[current]] + path
            directionVector = Actions.directionToVector(astarData.paths[current])
            current = (current[0] - directionVector[0], current[1] - directionVector[1])
        return True, path

    if current not in astarData.explored.keys():
        astarData.explored[current] = curr_g
    else:
        return False, []

    for successor in problem.getSuccessors(current):
        new_g, new_h = curr_g + successor[2], astar_heuristic(successor[0], astarData.goal)
        new_f = new_g + new_h
        new_node = (new_g, successor[0])

        if successor[0] not in astarData.explored.keys() or new_g < astarData.explored[successor[0]]:
            astarData.frontier.push(new_node, new_f)
            astarData.paths[successor[0]] = successor[1]

    return False, []

        
    

def astar_heuristic(current, goal):
    return util.manhattanDistance(current, goal)
