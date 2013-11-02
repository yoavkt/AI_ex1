# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
"""

import util
from game import Directions

class SearchProblem:
  """
  This class outlines the structure of a search problem, but doesn't implement
  any of the methods (in object-oriented terminology: an abstract class).
  
  You do not need to change anything in this class, ever.
  """
  
  def getStartState(self):
     """
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           


def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def FirstSearchHelper(problem, fringe):
  """
    Generic helper function for BFS and DFS  
  """

  # fringe: either Queue or Stack
  #         nodes in fringe will be (state, parent, direction)
  # closed: dictionary {state: node tuple}
  # path:   list of util.Directions
  closed = {}
  path = []

  root = problem.getStartState()
  fringe.push((root,None,Directions.STOP))
  
  while not fringe.isEmpty():
    node = fringe.pop()
    state = node[0]

    if problem.isGoalState(state):
      while node[2] != Directions.STOP:
        path.append(node[2]) 
        node = closed[node[1]] # Get parent node
      path.reverse()
      return path

    if not state in closed:
      closed[state] = node
      children = problem.getSuccessors(state)

      for child in children:
        if not closed.has_key(child[0]):
          #print 'Pushing ' +str((child[0], state, child[1]))#DEBUG
          fringe.push((child[0], state, child[1]))

  return None


def depthFirstSearch(problem):
  fringe = util.Stack()
  return FirstSearchHelper(problem, fringe)
  

def breadthFirstSearch(problem):
  fringe = util.Queue()
  return FirstSearchHelper(problem, fringe)
  
      
def uniformCostSearch(problem):
  #fringe = util.PriorityQueueWithFunction(lambda state_cost_pair : state_cost_pair[1])
  #return graphSearch(problem, fringe)
  return aStarSearch(problem)
  
def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  # Almost identical to FirstSearchHelper but making this really generic
  # would have made the code really cumbersome 
  #
  # fringe: util.PriorityQueue() with priority f(n)=g(n)+h(n)
  #         nodes in fringe will be (state, parent, direction)
  # closed: dictionary {state: node tuple}
  # path:   list of util.Directions
  # f,c,h:  dictionaries for cost functions
  closed = {}
  fringe = util.PriorityQueue()
  path = []
  c = {}
  h = {}
  f = {} 
    
  root = problem.getStartState()
  c[root] = 0
  h[root] = heuristic(root, problem)
  f[root] = c[root]+h[root]

  fringe.push((root, None, 'Stop'), f[root])

  while not fringe.isEmpty():
    node = fringe.pop()
    state = node[0]

    if problem.isGoalState(state):
      while node[2] != 'Stop':
        path.append(node[2])
        node = closed[node[1]]
      path.reverse()
      return path
    
    closed[state] = node
    children = problem.getSuccessors(state)
    childNode = None

    for child in children:
      if child[0] not in closed:
        c_child = c[state] + child[2]
      
        if not h.has_key(child[0]):
          h[child[0]] = heuristic(child[0], problem)

        if (not c.has_key(child[0])) or (c_child < c[child[0]]): 
          # node never checked or current path is better
          c[child[0]] = c_child
          f[child[0]] = c[child[0]] + h[child[0]]
          childNode = (child[0], state, child[1])
          fringe.push(childNode, f[child[0]])

  return None
    



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch