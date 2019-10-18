# pacmanAgents.py
# ---------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).
import collections

import util
from pacman import Directions
from game import Agent
from heuristics import *
import random

class RandomAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        actions = state.getLegalPacmanActions()
        # returns random action from all the valide actions
        return actions[random.randint(0,len(actions)-1)]

class OneStepLookAheadAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        legal = state.getLegalPacmanActions()
        # get all the successor state for these actions
        successors = [(state.generatePacmanSuccessor(action), action) for action in legal]
        # evaluate the successor states using scoreEvaluation heuristic
        scored = [(admissibleHeuristic(state), action) for state, action in successors]
        # get best choice
        bestScore = min(scored)[0]
        # get all actions that lead to the highest score
        bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
        # return random action from the list of the best actions
        return random.choice(bestActions)

class BFSAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write BFS Algorithm instead of returning Directions.STOP
        # get all legal actions for pacman
        legal = state.getLegalPacmanActions()
        # get all the successor state for these actions
        successors = collections.deque((state.generatePacmanSuccessor(action), action) for action in legal)
        # set flag to check whether generatePacmanSuccessor() is out of call. If it is, then flag = 1.
        none_flag = 0

        while successors:
            # define successor as queue
            # use popleft to pop the first element in the queue
            state, direction = successors.popleft()
            # if generatePacmanSuccessor() is out of call, break the loop to use Heuristic instead
            if none_flag == 1:
                break
            # if this state can win, then choose this way
            if state.isWin():
                return direction
            # get all legal actions of this state
            nextlegal = state.getLegalPacmanActions()
            # if this state has no legal action, then end up searching this way
            # search all successors of every legal action of next state
            for action in nextlegal:
                validSuccessor = state.generatePacmanSuccessor(action)
                # check whether generatePacmanSuccessor() is out of call
                if validSuccessor is None:
                    none_flag = 1
                    break
                # add successors to successors queue
                successors.append((validSuccessor, direction))

        # when generatePacmanSuccessor() is out of call, use Heuristic to evaluate all leave nodes
        if successors:
            # evaluate the successor states using scoreEvaluation heuristic
            scored = [(admissibleHeuristic(state), action) for state, action in successors]
            # get best choice
            bestScore = min(scored)[0]
            # get all actions that lead to the highest score
            bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
            # return the nearest leave node to the root from the list of the best actions
            return bestActions[0]


class DFSAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write BFS Algorithm instead of returning Directions.STOP
        # get all legal actions for pacman
        legal = state.getLegalPacmanActions()
        # get all the successor state for these actions
        successors = collections.deque((state.generatePacmanSuccessor(action), action) for action in legal)
        # set flag to check whether generatePacmanSuccessor() is out of call. If it is, then flag = 1.
        none_flag = 0

        while successors:
            # define successor as stack
            # use pop to pop the last element in the stack
            state, direction = successors.pop()
            # if generatePacmanSuccessor() is out of call, break the loop to use Heuristic instead
            if none_flag == 1:
                break
            # if this state can win, then choose this way
            if state.isWin():
                return direction
            # get all legal actions of this state
            nextlegal = state.getLegalPacmanActions()
            # if this state has no legal action, then end up searching this way
            # search all successors of every legal action of next state
            for action in nextlegal:
                validSuccessor = state.generatePacmanSuccessor(action)
                # check whether generatePacmanSuccessor() is out of call
                if validSuccessor is None:
                    none_flag = 1
                    break
                # add successors to successors stack
                successors.append((validSuccessor, direction))

        # when generatePacmanSuccessor() is out of call, use Heuristic to evaluate all leave nodes
        if successors:
            # evaluate the successor states using scoreEvaluation heuristic
            scored = [(admissibleHeuristic(state), action) for state, action in successors]
            # get best choice
            bestScore = min(scored)[0]
            # get all actions that lead to the highest score
            bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
            # return the nearest leave node to the root from the list of the best actions
            return bestActions[0]


class AStarAgent(Agent):
    # Initialization Function: Called one time when the game starts
    count = 0
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write BFS Algorithm instead of returning Directions.STOP
        # get all legal actions for pacman
        legal = state.getLegalPacmanActions()
        # use list to store all the successors
        successors = []
        # depth, the same as g(n), is the cost of the path since the start node
        depth = 1
        # store the data of successors' successors
        for action in legal:
            initSuccessor = state.generatePacmanSuccessor(action)
            cost = depth + admissibleHeuristic(initSuccessor)
            successors.append((cost, initSuccessor, action, depth))
        # set flag to check whether generatePacmanSuccessor() is out of call. If it is, then flag = 1.
        none_flag = 0

        while successors:
            # if generatePacmanSuccessor() is out of call, break the loop to use Heuristic instead
            if none_flag == 1:
                break
            # sort successors by the cost
            successors.sort(key = lambda x:x[0])
            # pop the successor with least cost
            cost, state, direction, depth = successors.pop(0)
            # since we're going to explore the successor's successors, depth add one
            depth += 1
            # if this state can win, then choose this way
            if state.isWin():
                return direction
            # get all legal actions of this state
            nextlegal = state.getLegalPacmanActions()
            # if this state has no legal action, then end up searching this way
            # search all successors of every legal action of next state
            for action in nextlegal:
                validSuccessor = state.generatePacmanSuccessor(action)
                # check whether generatePacmanSuccessor() is out of call
                if validSuccessor is None:
                    none_flag = 1
                    break
                cost = depth + admissibleHeuristic(validSuccessor)
                # add successors to successors list
                successors.append((cost, validSuccessor, direction, depth))

        # when generatePacmanSuccessor() is out of call, use Heuristic to evaluate all leave nodes
        if successors:
            # evaluate the successor states using scoreEvaluation heuristic
            scored = [(admissibleHeuristic(state), action) for a, state, action, b in successors]
            # get best choice
            bestScore = min(scored)[0]
            # get all actions that lead to the highest score
            bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
            # return the nearest leave node to the root from the list of the best actions
            return bestActions[0]