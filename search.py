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

def graphSearch(problem, fringe):
  start_state = problem.getStartState()
  fringe.push((start_state,0))
  closed = []
  tree = Graph()
  
  while not fringe.isEmpty():
    current = fringe.pop()
    current_state = current[0]
    current_cost = current[1]

    if problem.isGoalState(current_state):
      return tree.pathFromTo(start_state,current_state)

    if current_state not in closed:
      successors = problem.getSuccessors(current_state)
      for succ in successors:
          if tree.getVertex(succ[0]) == None:
            # Don't add an edge to a vertix already in the graph
            tree.addEdge(current_state,succ[0],succ[1:])
            fringe.push((succ[0],current_cost+succ[2]))
          
      closed.append(current_state)


def depthFirstSearch(problem):
  fringe = util.Stack()
  return graphSearch(problem, fringe)
  

def breadthFirstSearch(problem):
  fringe = util.Queue()
  return graphSearch(problem, fringe)
  
      
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
  fringe = util.PriorityQueueWithFunction(lambda state_cost_pair : state_cost_pair[1] + heuristic(state_cost_pair[0],problem))
  return graphSearch(problem, fringe)
    


class Graph:
  def __init__(self):
    self.vertList = {}
    self.numVertices = 0

  def addVertex(self,key):
    self.numVertices = self.numVertices + 1
    newVertex = Vertex(key)
    self.vertList[key] = newVertex
    return newVertex
    
  def getVertex(self,n):
    if n in self.vertList:
      return self.vertList[n]
    else:
      return None

  def __contains__(self,n):
    return n in self.vertList

  def addEdge(self,f,t,weight=0):
    if f not in self.vertList:
      self.addVertex(f)
    if t not in self.vertList:
      self.addVertex(t)
    self.vertList[f].addNeighbor(self.vertList[t].getId(), weight)
    self.vertList[t].setParent(f)


  def getVertices(self):
    return self.vertList.keys()

  def __iter__(self):
    return iter(self.vertList.values())

  def pathFromTo(self,f,t):
    """
    Assumes the graph is a tree!
    """
    path = []
    
    while f != t:
      p = self.vertList[t].getParent()
      path.insert(0,self.vertList[p].getWeight(t)[0])
      t = p

    return path


class Vertex:
  def __init__(self,key):
    self.id = key
    self.connectedTo = {}
    self.parent = None

  def addNeighbor(self,nbr,weight=0):
    self.connectedTo[nbr] = weight

  def hasNeighbor(self,nbr):
    return self.connectedTo.has_key(nbr)

  def setParent(self, parent):
    self.parent = parent

  def __str__(self):
    return str(self.id) + ' -> ' + str([x for x in self.connectedTo])

  def getConnections(self):
    return self.connectedTo.keys()

  def getId(self):
    return self.id

  def getWeight(self,nbr):
    return self.connectedTo[nbr]  

  def getParent(self):
    return self.parent





# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch