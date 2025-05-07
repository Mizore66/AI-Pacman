#---------------------#
# DO NOT MODIFY BEGIN #
#---------------------#

import logging

import util
from problems.q1b_problem import q1b_problem

def q1b_solver(problem: q1b_problem):
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
    # YOUR CODE HERE
    def __init__(self):
        self.frontier = util.PriorityQueue()
        self.goal = None
        self.explored = set()
        self.costs = {}
        self.paths = {}

def astar_initialise(problem: q1b_problem):
    # YOUR CODE HERE
    astarData = AStarData()
    start_state = problem.getStartState()
    astarData.goal = problem.startingGameState.getFood().asList()

    root_h, tiebreaker_h = astar_heuristic(problem.getStartState(), astarData.goal)
    astarData.frontier.push((0, start_state), (root_h, root_h, tiebreaker_h))
    astarData.costs[start_state] = 0
    astarData.paths[start_state] = []

    return astarData

def astar_loop_body(problem: q1b_problem, astarData: AStarData):
    curr_g, current = astarData.frontier.pop()

    if problem.isGoalState(current):
        path = []
        while current != problem.getStartState():
            path = [astarData.paths[current]] + path
            directionVector = Actions.directionToVector(astarData.paths[current])
            current = (current[0] - directionVector[0], current[1] - directionVector[1])
        return True, path

    if current not in astarData.explored:
        astarData.explored.add(current)
        astarData.costs[current] = curr_g

        for successor in problem.getSuccessors(current):
            new_g, new_h_pack = curr_g + successor[2], astar_heuristic(successor[0], astarData.goal)
            new_h, new_tiebreaker_h = new_h_pack
            new_f = new_g + new_h
            new_node =  (new_g, successor[0])

            if successor[0] not in astarData.costs.keys() or new_g < astarData.costs[successor[0]]:
                astarData.frontier.push(new_node, (new_f, new_h, new_tiebreaker_h))
                astarData.costs[successor[0]] = new_g
                astarData.paths[successor[0]] = successor[1]

    return False, None

def astar_heuristic(current, goals):
    if len(goals) == 1:
        return util.manhattanDistance(current, goals[0]), 0
    else:
        distances = sorted([util.manhattanDistance(current, goal) for goal in goals])
        return distances[0], distances[-1]
  
    