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

from pacman import Directions
from game import Agent
from heuristics import *
import random
import math

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

class RandomSequenceAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        self.actionList = [];
        for i in range(0,10):
            self.actionList.append(Directions.STOP);
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        possible = state.getAllPossibleActions();
        for i in range(0,len(self.actionList)):
            self.actionList[i] = possible[random.randint(0,len(possible)-1)];
        #print self.actionList
        tempState = state;
        for i in range(0,len(self.actionList)):
            if tempState.isWin() + tempState.isLose() == 0:
                tempState = tempState.generatePacmanSuccessor(self.actionList[i]);
            else:
                break;
        # returns random action from all the valide actions
        return self.actionList[0];

class HillClimberAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        self.actionList = [];
        for i in range(0, 5):
            self.actionList.append(Directions.STOP);
        self.maxscore = -float('inf')
        self.bestseq = []
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write Hill Climber Algorithm instead of returning Directions.STOP
        # get all legal actions for pacman
        possible = state.getAllPossibleActions();

        self.actionList[0] = possible[random.randint(0, len(possible) - 1)];

        none_flag = 0

        startState = state;
        tempState = state
        self.maxscore = gameEvaluation(startState, startState)

        while none_flag == 0:
            for i in range(0, len(self.actionList)-1):
                if tempState.isWin() + tempState.isLose() == 0:
                    possible = tempState.getAllPossibleActions();
                    scored = []
                    for action in possible:
                        currentState = tempState.generatePacmanSuccessor(action)
                        if currentState is None:
                            none_flag = 1
                            break
                        value = gameEvaluation(tempState, currentState)
                        scored.append((value,action))
                    if none_flag == 1:
                        break
                    scored = sorted(scored,key=lambda x : x[0])
                    bestmove = scored[-1][1]
                    self.actionList[i + 1] = bestmove
                    possibility = random.randint(0, 100)
                    if possibility > 50:
                        self.actionList[i+1] = possible[random.randint(0, len(possible) - 1)];
                        bestmove = self.actionList[i+1]

                    temp_tempState = tempState.generatePacmanSuccessor(bestmove)
                    if temp_tempState is None:
                        break
                    tempState = temp_tempState
                else:
                    break
            bestscore = gameEvaluation(startState, tempState)
            if bestscore > self.maxscore:
                self.maxscore = bestscore
                self.bestseq = self.actionList
            tempState = startState
            self.actionList = [];
            for i in range(0, 5):
                self.actionList.append(Directions.STOP);
            self.actionList[0]=possible[random.randint(0, len(possible) - 1)]
        return self.bestseq[0]



class GeneticAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        self.actionList = [];
        for i in range(0, 5):
            self.actionList.append(Directions.STOP);
        # population with size 5*8
        self.population = [self.actionList]*8
        self.possible = state.getAllPossibleActions()
        # initialize the first generation population randomly
        for j in range(len(self.population)):
            for i in range(len(self.actionList)):
                self.population[j][i] =self.possible[random.randint(0, len(self.possible) - 1)]
        self.bestseq = []
        return;

    #do rank selection to choose one individual
    def selectparent(self, candidates):
        x = random.uniform(0, 1)
        cumulative_probability = 0.0
        for seq, seq_probability in candidates:
            cumulative_probability += seq_probability
            if x < cumulative_probability:
                break
        return self.population[seq]

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write Genetic Algorithm instead of returning Directions.STOP
        # get all legal actions for pacman
        candidates = []
        #tempState = state
        none_flag = 0
        while none_flag == 0:
            # get candidates' fitness
            for j in range(len(self.population[:8])):
                tempState = state
                for i in range(len(self.actionList)):
                    if tempState.isWin() + tempState.isLose() == 0:
                        temp_tempState = tempState.generatePacmanSuccessor(self.population[j][i])
                        if temp_tempState is None:
                            none_flag = 1
                            break
                        tempState = temp_tempState
                    else:
                        break
                if none_flag == 1:
                    break
                candidates.append([j,gameEvaluation(state, tempState)])
            if none_flag == 1:
                break
            #rank  -- value from small to big
            candidates.sort(key=lambda item:item[1])
            self.bestseq = self.population[candidates[-1][0]]
            print self.bestseq
            print none_flag
            #possibility of being chosen
            rankSum = 1+2+3+4+5+6+7+8
            rank = 1.0
            for i in range(8):
                candidates[i][1] = rank/rankSum
                rank += 1
            #select next population
            #Use Rank Selection for picking each pair
            parent1 = self.selectparent(candidates=candidates)
            parent2 = self.selectparent(candidates=candidates)
            #crossover
            newborn = []
            newpopulation = []
            x = random.uniform(0, 100)
            while len(newpopulation)<8:
                if x <= 70:
                    for geneNum in range(5):
                        genePos = random.uniform(0, 100)
                        if genePos > 50:
                            newborn.append(parent1[geneNum])
                        else:
                            newborn.append(parent2[geneNum])
                    newpopulation.append(newborn)
                    newborn = []
                else:
                    newpopulation.append(parent1)
                    newpopulation.append(parent2)
            #mutation
            for i in range(len(newpopulation)):
                x = random.uniform(0, 100)
                if x <= 10:
                    mutaGene = random.randint(0, 4)
                    newpopulation[i][mutaGene] = self.possible[random.randint(0, len(self.possible) - 1)]
            # replace population
            self.population = newpopulation[:]
            candidates = []
        return self.bestseq[0]


class TreeNode():

    def __init__(self, action, parent=None):
        self.action = action
        self.parent = parent
        self.children = []
        self.full = False
        self.visit = 1
        self.reward = 0

    # add one child
    def addNewChild(self, action):
        newChild = TreeNode(action, self)
        self.children.append(newChild)

class MCTSAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        self.none_flag = False
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # TODO: write MCTS Algorithm instead of returning Directions.STOP
        none_flag = False
        #generate root
        rootnode = TreeNode(None)
        while none_flag is False:
            # select one node to expand
            selectnode = self.selection(rootnode, state)
            if selectnode is None:
                break
            # bp finished
            elif selectnode == 0:
                continue
            # do rollout stimulation to calculate the reward
            reward = self.stimulation(selectnode, state)
            if reward is None:
                break
            # bp finished
            elif reward == 0:
                continue
            # back propagation
            self.backPropagation(selectnode, reward)
        # choose the most visit child
        visit = []
        for child in rootnode.children:
            visit.append([child.action, child.visit])
        visit.sort(key=lambda item: item[1])
        return visit[-1][0]

    #selection
    def selection(self, node, state):
        find_flag = False
        while find_flag is False:
            # if this node is fully explored, we choose its best child to continue explore
            if node.full is True:
                # calculate the ucb value and choose the highest one
                ucbscore = []
                for child in node.children:
                    ucb = (child.reward/child.visit)+math.sqrt((2*math.log(node.visit))/child.visit)
                    ucbscore.append([child, ucb])
                ucbscore.sort(key=lambda item: item[1])
                node = ucbscore[-1][0]
            #if this node isn't fully explored, we choose it to explore
            else:
                #find_flag = True
                break
        return self.ExpandTheNode(node, state)

    ##MCTS Back Propogation
    def backPropagation(self, node, reward):
        while node:
            node.visit += 1
            node.reward += reward
            node = node.parent
        return

    ##MCTS Default Policy
    def stimulation(self, node, state):
        # trace back to get the path from root to node
        path = []
        tempNode = node
        while tempNode.parent:
            path.append(tempNode)
            tempNode = tempNode.parent
        path = path[::-1]
        tempState = state
        for node in path:
            if tempState.isWin() + tempState.isLose() == 0:
                temp_tempState = tempState.generatePacmanSuccessor(node.action)
                if temp_tempState is None:
                    self.none_flag = 1
                    self.backPropagation(node, gameEvaluation(state, tempState))
                    return None
                tempState = temp_tempState
            else:
                self.backPropagation(node, gameEvaluation(state, tempState))
                return 0
        newState = tempState
        #rollout in 5 times
        for j in range(5):
            if newState.isWin() + newState.isLose() == 0:
                legal = newState.getLegalPacmanActions()
                # no more action then stop the loop
                if not legal:
                    break
                # randomylywalk 5 times
                prevState = newState
                newState = newState.generatePacmanSuccessor(random.choice(legal))
                # when call is used up , stop the loop
                if newState is None:
                    newState = prevState
                    self.none_flag = True
                    break
            else:
                break
        reward = gameEvaluation(state, newState)
        return reward

    #expansion
    def ExpandTheNode(self, node, state):
        # trace back to get the path from root to node
        path = []
        tempNode = node
        while tempNode.parent is not None:
            path.append(tempNode)
            tempNode = tempNode.parent
        path = path[::-1]
        tempState = state
        for node in path:
            prevState = tempState
            tempState = tempState.generatePacmanSuccessor(node.action)
            if tempState is None:
                self.backPropagation(node, gameEvaluation(state, prevState))
                return None
            if tempState.isWin() + tempState.isLose() != 0:
                self.backPropagation(node, gameEvaluation(state, tempState))
                return 0
        # only expaned the child that not exist in the current node
        existaction = [child.action for child in node.children]
        legal = tempState.getLegalPacmanActions()
        for act in legal:
            if act not in existaction:
                node.addNewChild(act)
                break
        # update the node.full if it's fully explored
        if len(node.children) == len(legal):
            node.full = True
        return node.children[-1]

