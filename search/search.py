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
import heapq

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

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first
  [2nd Edition: p 75, 3rd Edition: p 87]
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm 
  [2nd Edition: Fig. 3.18, 3rd Edition: Fig 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  # Initialize a few queues to save the nodes
  explored = [] # Save nodes that have been visited
  # Save the nodes to be explored in the format (State, Action)
  frontier = [(problem.getStartState(), [])] 
  
  # Loop through all the nodes in the frontier
  while len(frontier) != 0:
      v, a = frontier.pop() # Get the last node
      explored.append(v) # Add to explored list to avoid redundant exploration
      
      # Get the nodes in the next depth
      successors = problem.getSuccessors(v)
      
      # Loop through the nodes in next depth
      for successor in successors:
          state, action, cost = successor
          
          if state in explored:
              continue
          # Join all the previous actions + current action
          path = a + [action]
          
          # Return the solution if the state is goal
          if problem.isGoalState(state):
              print(len(path))
              return path
          
          # If not goal, add the node and all actions taken to this node to frontier
          if len([tup for tup in frontier if tup[0] == state]) == 0:
              frontier.append((state, path))
  
  # Not applicable for the toy problem here, but added for completeness sake
  return False # If cannot find solution after visiting all the nodes
              
def breadthFirstSearch(problem):
  """
  Search the shallowest nodes in the search tree first.
  [2nd Edition: p 73, 3rd Edition: p 82]
  """
  "*** YOUR CODE HERE ***"
  
  """
  Everything is exactly the same as the depth first search.
  The only modification needed is to add the new expanded nodes to the END of the queue
  """
  
  # Initialize a few queues to save the nodes
  explored = [] # Save nodes that have been visited
  # Save the nodes to be explored in the format (State, Action)
  frontier = [(problem.getStartState(), [])] 
  
  # Loop through all the nodes in the frontier
  while len(frontier) != 0:
      v, a = frontier.pop() # Get the last node
      explored.append(v) # Add to explored list to avoid redundant exploration
      
      # Get the nodes in the next depth
      successors = problem.getSuccessors(v)
      
      # Loop through the nodes in next depth
      for successor in successors:
          state, action, cost = successor
          
          if state in explored:
              continue
          # Join all the previous actions + current action
          path = a + [action]
          
          # Return the solution if the state is goal
          if problem.isGoalState(state):
              return path
          
          
          # The only difference between breadth and depth first search.
          # Added the new expanded nodes to the END of the queue. Using pop so end of list = front of queue.
          if len([tup for tup in frontier if tup[0] == state]) == 0:
              frontier = [(state, path)] + frontier
  # Not applicable for the toy problem here, but added for completeness sake
  return False # If cannot find solution after visiting all the nodes
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  
  # Initialize a few queues to save the nodes
  explored = [] # Save nodes that have been visited
  """
  Save the nodes to be explored in the format (Cost, State, Action)
  Cost: Int
  State: Tuple
  Action: String
  """
  
  # Initialize the frontier
  frontier = [(0, problem.getStartState(), [])] 
  heapq.heapify(frontier) #Heapify it
  
  # Loop through all the nodes in the frontier
  while len(frontier) != 0:      
      c, v, a = heapq.heappop(frontier) # Get the last node (i.e., the node with the highest total cost)
      
      explored.append(v) # Add to explored list to avoid redundant exploration
         
      # Get the nodes in the next depth
      successors = problem.getSuccessors(v)
      
      # Loop through the nodes in next depth
      for successor in successors:
          pos, action, cost = successor
          
          # Skip if the node already explored
          if pos in explored:
              continue
          
          path = a + [action] # Join all the previous actions + current action
          
          # Return the solution if the state is goal
          if problem.isGoalState(pos):
              return path
          
          total_cost = c + cost # Add the cost to total cost
          
          # Find if the current node is in the frontier
          node = [tup for tup in frontier if tup[1] == pos]
          #If node does not exist, insert the node
          if len(node) == 0:
              heapq.heappush(frontier, (total_cost, pos, path))
          # If the node already exists but has a higher cost, replace it
          elif node[0][0] > total_cost:
              frontier.remove(node[0])
              heapq.heappush(frontier, (total_cost, pos, path))
              
  # Not applicable for the toy problem here, but added for completeness sake
  return False # If cannot find solution after visiting all the nodes

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  
  # Initialize a few queues to save the nodes
  explored = [] # Save nodes that have been visited
  """
  Save the nodes to be explored in the format (Total Cost, Path Cost, State, Action)
  Total Cost: f (i.e., g + h)
  Path Cost: g
  State: Tuple of location
  Action: String
  """
  # Initialize the frontier 
  frontier = [(0, 0, problem.getStartState(), [])] 
  heapq.heapify(frontier) #heapify it

  # Loop through all the nodes in the frontier
  while len(frontier) != 0:
      # Get the node with the smallest total cost
      tc, pc, v, a = heapq.heappop(frontier) # Get the last node (i.e., the node with the highest total cost)
      explored.append(v) # Add to explored list to avoid redundant exploration
      
      # Get the nodes in the next depth
      successors = problem.getSuccessors(v)
      
      # Loop through the nodes in next depth
      for successor in successors:
          state, action, cost = successor
          
          # Skip if the state is already in the explored list
          if state in explored:
              continue
          
          goal_dist = heuristic(state, problem) # Estimate the distance from the goal
          path = a + [action] # Join all the previous actions + current action
          
          # Return the solution if the state is goal
          if problem.isGoalState(state):
              return path
          
          path_cost = pc + cost # updated g
          total_cost = path_cost + goal_dist # g + h
          
          # Find if the current node is already in the frontier
          node = [tup for tup in frontier if tup[2] == state]

          #If node does not exist, insert the node
          if len(node) == 0:
              heapq.heappush(frontier, (total_cost, path_cost, state, path))
          # If the node does exist, but the total cost is higher than the current one, replace it
          elif node[0][0] > total_cost:
              frontier.remove(node[0])
              heapq.heappush(frontier, (total_cost, path_cost, state, path))

  # Not applicable for the toy problem here, but added for completeness sake
  return False # If cannot find solution after visiting all the nodes

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
