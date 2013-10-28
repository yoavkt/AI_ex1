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
           

def make_path_to_root(graph, end):
  """
    graph:  a dictionary whose keys are tuples (x,y) of integers and values are lists of tuples given by getSuccessors()
    return: returns a list of actions that lead pacman from the root of the search tree to the given end node
  """

  path = []
  end_while = False

  debug_counter = 0


  # add removal of values from graph dictionary?
  while end_while==False:
    end_while=True
    for k in graph.keys():
      for val in graph[k]:
        if val[0]==end:
          end_while = False
          path.insert(0,val[1])
          new_end = k

          if debug_counter<12: #DEBUG
            print graph
            print 'k: ' + str(k) + '\n'
            print 'val: ' + str(val) + '\n' 
            print 'path: ' + str(path) + '\n'
            print 'end: ' +str(end) + '\n'
            print 'new_end: ' + str(new_end) +'\n'
            debug_counter+=1

    end = new_end
  
  return path


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
  from game import Directions
  n = Directions.NORTH
  e = Directions.EAST
  s = Directions.SOUTH
  w = Directions.WEST
  fringe.push(problem.getStartState())
  closed = [problem.getStartState()]
  graph = {}
  
  while not fringe.isEmpty():
    current=fringe.pop()
    graph[current]=[] #create new node
    if problem.isGoalState(current):
      return make_path_to_root(graph,current)
    else:    
      successors = problem.getSuccessors(current)
      for succ in successors:
        if succ[0] not in closed:
          fringe.push(succ[0])
          closed.append(succ[0])
          graph[current].append(succ)

def depthFirstSearch(problem):
  ds= util.Stack()
  return graphSearch(problem, ds)
  """
  Search the deepest nodes in the search tree first [p 85].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def breadthFirstSearch(problem):
  ds= util.Queue()
  return graphSearch(problem, ds)
  util.raiseNotDefined()
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  util.raiseNotDefined()
    
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch