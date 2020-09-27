#Sukhmani Choudhry
#Schoud9@emory.edu
# /*THIS  CODE  WAS MY OWN WORK , IT WAS  WRITTEN  WITHOUT  CONSULTING  ANY SOURCES  OUTSIDE  OF  THOSE  APPROVED  BY THE  INSTRUCTOR. _Sukhmani_Choudhry_*/


# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util

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

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"

    start = problem.getStartState()
    return recursiveDFS(problem, start, [], [start])


def recursiveDFS(problem, current, directions, visited):
    ## base case: reached the goal
    if problem.isGoalState(current):
        return directions

    for point in problem.getSuccessors(current):
        # ignore points we already tried
        if point[0] in visited:
            continue

        newDirections = directions[:]
        newDirections.append(point[1])
        visited.append(point[0])

        answer = recursiveDFS(problem, point[0], newDirections, visited)

        # We found an answer!
        if answer != None:
            return answer

    # returns none if there's no to the goal down this way
    return None


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    start = problem.getStartState()
    return recursiveBFS(problem, [(start, [])], [start])


def recursiveBFS(problem, fringes, visited):
    newFringe = []

    for point in fringes:
        for successor in problem.getSuccessors(point[0]):
            ## skip visited successors
            if successor[0] in visited:
                continue

            visited.append(successor[0])

            ## Copy directions to get to this node
            newDirections = point[1][:]

            ## Add it to the next fringe layer
            newDirections.append(successor[1])
            newFringe.append((successor[0], newDirections))

            ## Check if we've reached the goal
            if problem.isGoalState(successor[0]):
                return newDirections

    return recursiveBFS(problem, newFringe, visited)



def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    visited = [start]

    ## Fringe holds the outer unvisited points, directions to that point, and cost of getting there.
    fringe = util.PriorityQueue()
    fringe.push((start, [], 0), 0)

    while not fringe.isEmpty():
        curr = fringe.pop()
        currPoint = curr[0]
        currDirections = curr[1]
        currCost = curr[2]

        for successor in problem.getSuccessors(curr[0]):
            if successor[0] in visited:
                continue

            ## check if we've reached the goal
            if problem.isGoalState(successor[0]):
                currDirections.append(successor[1])
                return currDirections

            visited.append(successor[0])

            ## update directions & cost, then add to the fringe
            newDirections = currDirections[:]
            newDirections.append(successor[1])

            newCost = currCost + successor[2]
            fringe.push((successor[0], newDirections, newCost), newCost)

    return None




def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    visited = [start]

    ## Fringe holds the outer unvisited points, directions to that point, and cost of getting there.
    fringe = util.PriorityQueue()
    fringe.push((start, [], 0), 0)

    while not fringe.isEmpty():
        curr = fringe.pop()
        currPoint = curr[0]
        currDirections = curr[1]
        currCost = curr[2]

        for successor in problem.getSuccessors(curr[0]):
            if successor[0] in visited:
                continue

                ## check if we've reached the goal
            if problem.isGoalState(successor[0]):
                currDirections.append(successor[1])
                return currDirections

            visited.append(successor[0])

            ## update directions & cost, then add to the fringe
            newDirections = currDirections[:]
            newDirections.append(successor[1])

            newCost = currCost + successor[2]
            fringe.push((successor[0], newDirections, newCost), newCost + heuristic(successor[0], problem))

    return None



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
