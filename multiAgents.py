"""
In multiAgents.py, you will implement the Minimax
adversarial search algorithm, as well as the alpha-beta
pruning variant, to be called in multi-agent Pacman games.
"""

from util import manhattanDistance
from game import Directions
import random, util

from game import Agent


#########################################################
# Example ReflexAgent implementation                    #
#########################################################

class ReflexAgent(Agent):
    """
      A reflex agent chooses an action at each choice point by examining
      its alternatives via a state evaluation function.

      The code below is provided as an example.  You are welcome to change
      it in any way you see fit, so long as you don't touch our method
      headers.
    """


    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        return successorGameState.getScore()

def scoreEvaluationFunction(currentGameState):
    """
      This default evaluation function just returns the score of the state.
      The score is the same one displayed in the Pacman GUI.

      This evaluation function is meant for use with adversarial search agents
      (not reflex agents).
    """
    return currentGameState.getScore()


#########################################################
# Minimax search - put your code for Questions 6-7 here #
#########################################################

class MultiAgentSearchAgent(Agent):
    """
      This class provides some common elements to all of your
      multi-agent searchers.  Any methods defined here will be available
      to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

      You *do not* need to make any changes here, but you can if you want to
      add functionality to all your adversarial search agents.  Please do not
      remove anything, however.

      Note: this is an abstract class: one that should not be instantiated.  It's
      only partially specified, and designed to be extended.  Agent (game.py)
      is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)
    def isTerminal(self, state, depth, agent):
        return depth == self.depth or \
               state.isWin() or \
               state.isLose() or \
               state.getLegalActions(agent) == 0

    # is this agent pacman
    def isPacman(self, state, agent):
        return agent % state.getNumAgents() == 0

class MinimaxAgent(MultiAgentSearchAgent):
    """
      Your minimax agent (question 6)
    """

    def getAction(self, gameState):
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
        "*** YOUR CODE HERE ***"
        bestScore, bestMove = self.maxFunction(gameState, self.depth)

        return bestMove

    def maxFunction(self, gameState, depth):
        if depth == 0 or gameState.isWin() or gameState.isLose():
            return self.evaluationFunction(gameState), "noMove"

        moves = gameState.getLegalActions()
        scores = [self.minFunction(gameState.generateSuccessor(self.index, move), 1, depth) for move in moves]
        bestScore = max(scores)
        indices = [index for index in range(len(scores)) if scores[index] == bestScore]
        return bestScore, moves[indices[0]]

    def minFunction(self, gameState, agent, depth):
        if depth == 0 or gameState.isWin() or gameState.isLose():
            return self.evaluationFunction(gameState), "noMove"
        moves = gameState.getLegalActions(agent)  # get legal actions.
        scores = []
        if (agent != gameState.getNumAgents() - 1):
            scores = [self.minFunction(gameState.generateSuccessor(agent, move), agent + 1, depth) for move in moves]
        else:
            scores = [self.maxFunction(gameState.generateSuccessor(agent, move), (depth - 1))[0] for move in moves]
        minScore = min(scores)
        worstIndices = [index for index in range(len(scores)) if scores[index] == minScore]
        return minScore, moves[worstIndices[0]]

        util.raiseNotDefined()

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
      Your minimax agent with alpha-beta pruning (question 7)
    """
    def getAction(self, gameState):
        """
          Returns the minimax action using self.depth and self.evaluationFunction
        """
        def dispatch(state, depth, agent, A=float("-inf"), B=float("inf")): #inf is infinite
            if agent == state.getNumAgents():
                depth += 1
                agent = 0
            if self.isTerminal(state, depth, agent):  # get the end
                return self.evaluationFunction(state), None
            if self.isPacman(state, agent):
                return getValue(state, depth, agent, A, B, float('-inf'), max)
            else:
                return getValue(state, depth, agent, A, B, float('inf'), min)

        def getValue(state, depth, agent, A, B, i, j): #i j stand for scope
            bestScore = i
            bestAction = None

            for action in state.getLegalActions(agent):
                successor = state.generateSuccessor(agent, action)
                score, _ = dispatch(successor, depth, agent + 1, A, B)
                bestScore, bestAction = j((bestScore, bestAction), (score, action))

                if self.isPacman(state, agent):
                    if bestScore > B: #when current best score is greater than inf
                        return bestScore, bestAction
                    A = j(A, bestScore)
                else:
                    if bestScore < A: #when current best score is smaller than -inf
                        return bestScore, bestAction
                    B = j(B, bestScore)

            return bestScore, bestAction
        _, action = dispatch(gameState, 0, 0)
        return action
        util.raiseNotDefined()
