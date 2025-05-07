#---------------------#
# DO NOT MODIFY BEGIN #
#---------------------#

import logging

import util
from problems.q1c_problem import q1c_problem

#-------------------#
# DO NOT MODIFY END #
#-------------------#

def q1c_solver(problem: q1c_problem):
    # YOUR CODE HERE
    astarData = bfs_initialise(problem)
    num_expansions = 0
    terminate = False
    while not terminate:
        num_expansions += 1
        terminate, result = bfs(problem, astarData)
    print(f'Number of node expansions: {num_expansions}')
    return result

#-------------------#
# DO NOT MODIFY END #
#-------------------#

from game import Actions

class Data:
    def __init__(self):
        self.frontier = util.PriorityQueue()
        self.explored = set()

def bfs_initialise(problem: q1c_problem):
    data = Data()
    start_state = problem.getStartState()

    data.frontier.push((start_state, []), len(start_state[1]))
    return data

def bfs(problem: q1c_problem, data: Data):
   while not data.frontier.isEmpty():
       current, path = data.frontier.pop()

       if problem.isGoalState(current):
           return True, path

       if current not in data.explored:
           data.explored.add(current)
           for successor, action, _ in problem.getSuccessors(current):
               if successor in data.explored:
                   continue
               new_path = path + [action]
               data.frontier.push((successor, new_path), len(successor[1]))

           return False, []

   return False, None


