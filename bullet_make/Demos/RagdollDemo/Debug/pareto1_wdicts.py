# -*- coding: utf-8 -*-
print "Open the Pod Bay doors, Hal"
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math as m
import os.path
import sys
import time
import csv
import pickle

NUM_INPUTS_ANN = 4
NUM_OUTPUTS_ANN = 13
"""
## classes, new stuff

POP_SIZE = 10
BIG_NUMBER = 999999999

class Individual:

    def __init__(self):
        self.id = None
        self.categorizationError = BIG_NUMBER
        self.predictionError = BIG_NUMBER
        self.ANN = MatrixCreate2(NUM_INPUTS_ANN, NUM_OUTPUTS_ANN)

    def Copy(self):
        newIndividual = MatrixCreate2(NUM_INPUTS_ANN, NUM_OUTPUTS_ANN)
        for rowidx in range(0, NUM_INPUTS_ANN):
            for colidx in range(0, NUM_OUTPUTS_ANN):
                newIndividual[rowidx][colidx] = self.ANN[rowidx][colidx]
        return newIndividual

    def getMutatedVersion(self):
        newInd = self.Copy()
        MatrixRandomize(newInd.ANN)
        return newInd

    def isNonDominated(self, otherIndividuals):
        # logic for defining nondom
        pass

oldpopulation = [Individual() for i in range(0, POP_SIZE)]
for generation in range(0, 10):
    oldpopulation = [individual.Copy() for individual in oldpopulation]
    nondoms = [individual for individual in oldpopulation
               if individual.isNotDominated(oldpopulation)]
    newpopulation = nondoms
    newpopulation = [(np.random.choice(nondoms)).getMutatedVersion() for i in range(0, POP_SIZE)]
    oldpopulation = newpopulation


## end classes, new stuff
"""

def MatrixCreate(size):  #Creates Matrix of zeros
    return np.zeros((size), dtype=float)

def MatrixCreate2(size, generations):
    return np.zeros((size, generations), dtype=float)

def MatrixRandomize(v):      #Creates Random Matrix
    rows = len(v)
    columns = len(v[0])
    for i in range(rows):
        for j in range(columns):
            v[i][j] = np.random.uniform(-1, 1)
    return v

def Send_Values_ToFile_two(data, filename):
    with open(filename, "wb") as output:
        writer = csv.writer(output, lineterminator=',')
        for val in data:
            writer.writerow([val])

def Send_Values_ToFile(data, filename):
    with open(filename, 'wb') as csvfile:
        testwriter = csv.writer(csvfile, delimiter = ',')
        testwriter.writerows(data)

def matrixPerturb(v, prob):
    p = np.copy(v)
    for i in range(len(v)):
        for j in range(len(v[0])):
            if prob > np.random.rand():
                p[i][j] = np.random.uniform(-1, 1);
    return p

def runner(i):
    """Runs the Robot 4 times, 1 time for each position and size"""
    if i == 0:
        position_size = [3.0, 0.0, 3.0, 2.0, 2.0, 0.5]
        Send_Values_ToFile_two(position_size, "position_size.csv")
        os.system('./AppRagdollDemo')
        #print position_size
    if i == 1:
        position_size = [-3.0, 0.0, 3.0, 2.0, 2.0, 0.5]
        Send_Values_ToFile_two(position_size, "position_size.csv")
        os.system('./AppRagdollDemo')
        #print position_size
    if i == 2:
        position_size = [3.0, 0.0, 3.0, 1.0, 1.0, -0.5]
        Send_Values_ToFile_two(position_size, "position_size.csv")
        os.system('./AppRagdollDemo')
        #print position_size
    if i == 3:
        position_size = [-3.0, 0.0, 3.0, 1.0, 1.0, -0.5]
        Send_Values_ToFile_two(position_size, "position_size.csv")
        os.system('./AppRagdollDemo')
        #print position_size
    return

def save_obj(obj, name ):
    with open( name + '.pickle', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def pos_size_fit(parent):
    """Finds the Categorize and Prediction values for the objective function"""
    Send_Values_ToFile(parent, "weights_Matrix.csv")
    cat_fit = [0.0, 0.0, 0.0, 0.0]
    cor_fit = [0.0, 0.0, 0.0, 0.0]
    g = open('g_vector.txt', 'r')
    m = open('m_vector.txt', 'r')
    g_v = float(g.read().rstrip().split(',')[0])
    m_v = float(m.read().rstrip().split(',')[0])
    print g, m
    for i in range(0,4):
        runner(i)
        f = open('categorize.csv', 'r')
        p = open('uncertainty.csv', 'r')
        cat_fit[i] = float(f.read().rstrip().split(',')[0])
        cor_fit[i] = float(p.read().rstrip().split(',')[0])
    cat = sum(cat_fit) / 4.0
    uncert = sum(cor_fit) / 4.0
    #print cat_fit
    #print pred_fit
    print "cat/uncert =", g_v, m_v
    #print "cat/uncert =", cat, uncert
    return cat, uncert

def vectors(p, cat, uncert, parent, cat_vector, cor_vector):
    """ Takes the values from each individual and adds them to their
        respective vector, either categorization value or prediction value"""

    cat_vector[0][p] = cat
    cor_vector[0][p] = uncert
    return cat_vector, cor_vector

def dominate(EvoA, bool_dict, g):
    """ Goes through the vector and dominates individuals by removing them and
        adding them to the nondominated key of the algorithm dictionary, which
        then becomes the new current population in the next generation """

    EvoA[g]['curr_pop']['nondominated'] = {}
    EvoA[g]['new_pop'] = {}
    EvoA[g]['new_pop']['pop'] = {}
    for e in range(0, 10):
        EvoA[g]['curr_pop']['dominated'][e] = {}
        if bool_dict[e] == 'F':   #if it's not T it is F
            EvoA[g]['curr_pop']['nondominated'][e] = deepCopyIndividual(EvoA[g]['curr_pop']['pop'][e])
            EvoA[g]['new_pop']['pop'][e] = deepCopyIndividual(EvoA[g]['curr_pop']['pop'][e])
        else:
            if bool_dict[e] == 'F':
                EvoA[g]['curr_pop']['dominated'][e] = deepCopyIndividual(EvoA[g]['curr_pop']['pop'][e])
                del EvoA[g]['curr_pop']['pop'][e]
                #continue

    return EvoA, bool_dict

def pareto_vector(vector1, vector2, EvoA, g):
    """ Creates a vector of True or Falses depending if the individual is dominated by another
        individual or not. *** T = dominated | F = nondominated ***   """

    bool_dict = {}
    EvoA[g]['curr_pop']['dominated'] = {}
    for i in range(0, 10):
        for j in range(0, 10):
            if i != j:
                if ((vector1[0][i] > vector1[0][j]) and (vector2[0][i] < vector2[0][j])):
                    #print vector1[0][i], vector1[0][j]   #to check if values are correct
                    #print vector2[0][i], vector2[0][j]
                    bool_dict[i] = 'T'
                    break
                    #EvoA[g]['curr_pop']['dominated'][i] = EvoA[g]['curr_pop']['pop'][i]
                    #del EvoA[g]['curr_pop']['pop'][i]

                if ((vector1[0][i] == vector1[0][j]) and (vector2[0][i] == vector2[0][j])):
                    A = np.random.uniform(0, 1)
                    if A > 0.5:
                        #print "Equal Cat and Pred"
                        bool_dict[i] = 'T'
                    else:
                        bool_dict[i] = 'F'
                else:
                    #EvoA[g]['curr_pop']['nondominated'] = EvoA[g]['curr_pop']['pop']
                    #EvoA[g]['new_pop']['pop'] = EvoA[g]['curr_pop']['pop']
                    bool_dict[i] = 'F'

    print bool_dict    #Comment this out to get rid of the line of true and false in printout

    EvoA, bool_dict = dominate(EvoA, bool_dict, g)

    return EvoA


def initial(g, EvoA, cat_vector, cor_vector, rows, columns):
    """" initial population """
    EvoA[g]['curr_pop'] = {}
    EvoA[g]['curr_pop']['pop'] = {}
    for p in range(0, 10):
        parent = MatrixCreate2(rows, columns)
        parent = MatrixRandomize(parent)
        parent_cat, parent_cor = pos_size_fit(parent)
        EvoA[g]['curr_pop']['pop'][p] = deepCopyIndividual(parent)
        cat_vector, cor_vector = vectors(p, parent_cat, parent_cor, parent, cat_vector, cor_vector)
    return EvoA, cat_vector, cor_vector,

def deepCopyIndividual(individual):
    newIndividual = MatrixCreate2(NUM_INPUTS_ANN, NUM_OUTPUTS_ANN)
    for rowidx in range(0, NUM_INPUTS_ANN):
        for colidx in range(0, NUM_OUTPUTS_ANN):
            newIndividual[rowidx][colidx] = individual[rowidx][colidx]
    return newIndividual

def secondary(g, EvoA, cat_vector, cor_vector):
    """ proceding populations through generations greater than 1 """
    EvoA[g]['new_pop'] = {}
    EvoA[g]['new_pop']['pop'] = {}
    EvoA[g]['curr_pop'] = {}
    EvoA[g]['curr_pop']['pop'] = {}
    global parent
    for k in range(0, 10):
        EvoA[g]['new_pop']['pop'][k] = {}
        nonDomKeys = EvoA[g-1]['new_pop']['pop'].keys()
        #print 'num of nondominated=',len(temp_list)       #shows the num of nondoms in each gen
        for key in nonDomKeys:
            if k == key:
                parent = deepCopyIndividual(EvoA[g-1]['new_pop']['pop'][key])
            else:
                temp = np.random.choice(nonDomKeys, 1, replace=True)
                temp = int(temp)
                parent = deepCopyIndividual(EvoA[g-1]['new_pop']['pop'][temp])
                parent = matrixPerturb(parent, 0.05)

        EvoA[g]['curr_pop']['pop'][k] = parent
        parent_cat, parent_cor = pos_size_fit(parent)
        cat_vector, cor_vector = vectors(k, parent_cat, parent_cor, parent, cat_vector, cor_vector)
    return EvoA, cat_vector, cor_vector


def evolve(rows, columns, prob, generations):
    """ Runs the whole program, evolves robot over G number of generations using
        pareto-front / multidimensional objectives"""
    EvoA = {}
    EvoA['Most_Recent'] = {}
    cor_vector = MatrixCreate2(1, 10)   #Predict Values
    cat_vector = MatrixCreate2(1, 10)   #Categorize values

    for g in range(0, generations):
        EvoA[g] = {}

    add = 0.0
    for g in range(0, generations):
        if g == 0:
            EvoA, cat_vector, cor_vector = initial(g, EvoA, cat_vector, cor_vector, rows, columns)
            #plt.scatter(cat_vector, pred_vector)
            EvoA = pareto_vector(cat_vector, cor_vector, EvoA, g)
            EvoA['Most_recent'] = EvoA[g]['curr_pop']

        if g > 0:
            print 'got through to generation %0.1f' % g
            EvoA, cat_vector, cor_vector = secondary(g, EvoA, cat_vector, cor_vector)
            #plt.scatter(cat_vector, pred_vector)
            EvoA = pareto_vector(cat_vector, cor_vector, EvoA, g)
            del EvoA['Most_recent']
            EvoA['Most_recent'] = EvoA[g]['curr_pop']

        #plt.show()
        add = add + 1
        print (add + 1, generations)

        save_obj(EvoA['Most_recent'], 'recent_robot')  #Saves most recent dictionary

        #for i in EvoA["curr_pop"]["nondominated"].keys():
        #s    EvoA["new_pop"]["nondominated"][i] = EvoA["curr_pop"]["nondominated"][i]

        #if len(EvoA[g]['curr_pop']['nondominated'].keys()) > 10:
        #    print len(EvoA[g]['curr_pop']['nondominated'])
        #    print "Something is wrong with the EvoA Dictionary in Python (out of range)"

        #print "number of nondominated individuals =", len(EvoA[g]['new_pop']['pop'])
        #print "number of dominated individuals =", len(EvoA[g]['curr_pop']['dominated'])

    return EvoA


Evo = evolve(4, 25, 0.05, 2)

#print "My god, it's full of stars..."
