"""
In astar_search.py, you will implement the A* search algorithms 
and an alternate heuristic for it, which are called by
Pacman agents (in searchAgents.py).
"""

import util
import searchAgents

#########################################################
# Pre-built heuristics; do not modify these!            #
#########################################################

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

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


#########################################################
# Code you need to edit for q4, q5 starts here          #
#########################################################
def getSolution(goalVertex, parentNodes, parentActions):
    '''
    This is a helper function to retrieve a solution path,
    because in the current implementation we are saving
    used memory by storing only one parent array
    - this is a hint from ACM ICPC experience
    '''

    resPath = []
    v = goalVertex

    # Just going up by parents
    while v in parentNodes:
        resPath.append(parentActions[v])
        v = parentNodes[v]

    # Now we need to reverse the path,
    # because we started from the goal
    resPath.reverse()

    return resPath

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    priorityQueueAStar = util.PriorityQueue()
    done = set()  # store passed state
    startNode = (problem.getStartState(), 0, [])  # start contains node, cost, and path
    initial_cost = 0 + heuristic(startNode[0], problem)
    priorityQueueAStar.push(startNode, initial_cost)

    while not priorityQueueAStar.isEmpty():
        (node, cost, path) = priorityQueueAStar.pop()
        if problem.isGoalState(node):
            return path
        if not node in done:
            done.add(node)
            for next_node, next_action, next_cost in problem.getSuccessors(node):
                totalCost = cost + next_cost
                totalPath = path + [next_action]
                totalState = (next_node, totalCost, totalPath)
                totalCost +=  heuristic(totalState[0], problem)
                priorityQueueAStar.push(totalState, totalCost)


    util.raiseNotDefined()

def foodHeuristic(state, problem):
    """
    Your heuristic for the FoodSearchProblem goes here.

    This heuristic must be consistent to ensure correctness.  First, try to come
    up with an admissible heuristic; almost all admissible heuristics will be
    consistent as well.

    If using A* ever finds a solution that is worse uniform cost search finds,
    your heuristic is *not* consistent, and probably not admissible!  On the
    other hand, inadmissible or inconsistent heuristics may find optimal
    solutions, so be careful.

    The state is a tuple ( pacmanPosition, foodGrid ) where foodGrid is a Grid
    (see game.py) of either True or False. You can call foodGrid.asList() to get
    a list of food coordinates instead.

    If you want access to info like walls, capsules, etc., you can query the
    problem.  For example, problem.walls gives you a Grid of where the walls
    are.

    If you want to *store* information to be reused in other calls to the
    heuristic, there is a dictionary called problem.heuristicInfo that you can
    use. For example, if you only want to count the walls once and store that
    value, try: problem.heuristicInfo['wallCount'] = problem.walls.count()
    Subsequent calls to this heuristic can access
    problem.heuristicInfo['wallCount']
    """
    "*** YOUR CODE HERE ***"
    position, foodGrid = state
    distance = [0]
    for food in foodGrid.asList():
        distance.append(util.manhattanDistance(position, food))

    furthestDistance = max(distance)

    return furthestDistance
# Abbreviations
astar = aStarSearch
