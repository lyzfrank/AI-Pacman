"""
In uninformed_search.py, you will implement generic search algorithms 
(using uninformed search approaches) which are called by
Pacman agents (in searchAgents.py).
"""

import util
import searchAgents


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]



def graphSearch(problem, frontier):
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    explored = []
    frontier.push([(problem.getStartState(), "Stop", 0)])

    while not frontier.isEmpty():
        # print "frontier: ", frontier.heap
        path = frontier.pop()
        # print "path len: ", len(path)
        # print "path: ", path

        s = path[len(path) - 1]
        s = s[0]
        # print "s: ", s
        if problem.isGoalState(s):
            # print "FOUND SOLUTION: ", [x[1] for x in path]
            return [x[1] for x in path][1:]

        if s not in explored:
            explored.append(s)
            # print "EXPLORING: ", s

            for successor in problem.getSuccessors(s):
                # print "SUCCESSOR: ", successor
                if successor[0] not in explored:
                    successorPath = path[:]
                    successorPath.append(successor)
                    # print "successorPath: ", successorPath
                    frontier.push(successorPath)
                    # else:
                    # print successor[0], " IS ALREADY EXPLORED!!"

    return []

def generic_search(problem, fringe, add_to_fringe_fn):
    closed = set()
    start = (problem.getStartState(), 0, [])  # (node, cost, path)
    add_to_fringe_fn(fringe, start, 0)

    while not fringe.isEmpty():
        (node, cost, path) = fringe.pop()

        if problem.isGoalState(node):
            return path

        if not node in closed:
            closed.add(node)

            for child_node, child_action, child_cost in problem.getSuccessors(node):
                new_cost = cost + child_cost
                new_path = path + [child_action]
                new_state = (child_node, new_cost, new_path)
                add_to_fringe_fn(fringe, new_state, new_cost)


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())

    stackDFS = util.Stack()
    done = set() # store passed node
    startNode = (problem.getStartState(), 0, [])  # (node, cost, path)
    stackDFS.push(startNode)

    while not stackDFS.isEmpty():
        (node, cost, path) = stackDFS.pop()
        if problem.isGoalState(node):
            return path
        if not node in done:
            done.add(node)
            for next_node, next_action, next_cost in problem.getSuccessors(node):
                totalCost = cost + next_cost
                totalPath = path + [next_action]
                totalState = (next_node, totalCost, totalPath)
                stackDFS.push(totalState)

    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    queueBFS = util.Queue()
    done = set()  # store passed state
    startNode = (problem.getStartState(), 0, [])  # start contains node, cost, and path
    queueBFS.push(startNode)

    while not queueBFS.isEmpty():
        (node, cost, path) = queueBFS.pop()
        if problem.isGoalState(node):
            return path
        if not node in done:
            done.add(node)
            for next_node, next_action, next_cost in problem.getSuccessors(node):
                totalCost = cost + next_cost
                totalPath = path + [next_action]
                totalState = (next_node, totalCost, totalPath)
                queueBFS.push(totalState)

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    priorityQueueUCS = util.PriorityQueue()
    done = set()  # store passed state
    startNode = (problem.getStartState(), 0, [])  # start contains node, cost, and path
    priorityQueueUCS.push(startNode,0)

    while not priorityQueueUCS .isEmpty():
        (node, cost, path) = priorityQueueUCS .pop()
        if problem.isGoalState(node):
            return path
        if not node in done:
            done.add(node)
            for next_node, next_action, next_cost in problem.getSuccessors(node):
                totalCost = cost + next_cost
                totalPath = path + [next_action]
                totalState = (next_node, totalCost, totalPath)
                priorityQueueUCS.push(totalState,totalCost)

    util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
ucs = uniformCostSearch
