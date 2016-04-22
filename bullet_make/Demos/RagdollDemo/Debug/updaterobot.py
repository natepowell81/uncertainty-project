
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
NUM_OUTPUTS_ANN = 26

## classes, new stuff

def MatrixCreate2(size, generations):
    return np.zeros((size, generations), dtype=float)
    
def MatrixRandomize(v):      #Creates Random Matrix
    rows = len(v)
    columns = len(v[0])
    for i in range(rows):
        for j in range(columns):
            v[i][j] = np.random.uniform(-1, 1)
    return v

def matrixPerturb(v, prob):
    p = np.copy(v)
    for i in range(len(v)):
        for j in range(len(v[0])):
            if prob > np.random.rand():
                p[i][j] = np.random.uniform(-1, 1);
    return p

def Send_Values_ToFile(data, filename):
    with open(filename, 'wb') as csvfile:
        testwriter = csv.writer(csvfile, delimiter = ',')
        testwriter.writerows(data)
        
def Send_Values_ToFile_two(data, filename):
    with open(filename, "wb") as output:
        writer = csv.writer(output, lineterminator=',')
        for val in data:
            writer.writerow([val])

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

filename = "weights_Matrix.csv"

POP_SIZE = 10
BIG_NUMBER = 999999999


class Individual:
    
    def __init__(self):
        self.id = None
        self.categorizationError = BIG_NUMBER
        self.predictionError = BIG_NUMBER
        self.ANN = MatrixCreate2(NUM_INPUTS_ANN, NUM_OUTPUTS_ANN)
        self.random = MatrixRandomize(self.ANN)        
        self.bool = 'F' #nondominated
        self.age = 0.0  #initial age
        self.fit = 0.0 #initial fitness
        
    def Copy(self):
        newIndividual = MatrixCreate2(NUM_INPUTS_ANN, NUM_OUTPUTS_ANN)
        for rowidx in range(0, NUM_INPUTS_ANN):
            for colidx in range(0, NUM_OUTPUTS_ANN):
                newIndividual[rowidx][colidx] = self.ANN[rowidx][colidx]
        return newIndividual

    def getMutatedVersion(self):
        newInd = self.Copy()
        newInd = matrixPerturb(newInd, 0.05)
        return newInd

    def isNonDominated(self, otherIndividuals):
        pass
        
    def getValues(self):
        return self.ANN
    
    def errors(self, cat_error, pred_error):
        self.categorizationError = cat_error
        self.predictionError = pred_error
        return 
        
    def getErrors(self):
        return self.categorizationError, self.predictionError

#oldpopulation = [Individual() for i in range(0, POP_SIZE)]


def run(population):
    cat_list = MatrixCreate2(1, len(population))
    corr_list = MatrixCreate2(1, len(population))
    for i in range(0, len(population)):
        parent = population[i].getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        cat_list_2 = MatrixCreate2(1, 4)
        corr_list_2 = MatrixCreate2(1, 4)
        for j in range(0, 4):
            runner(j)
            g = open('categorize.csv', 'r')
            
            gvect = open('g_vector.txt', 'r')
            gvect_list = []
            for line in gvect:
                gvect_list.append(float(line))
            mvect = open('m_vector.txt', 'r')
            mvect_list = []
            for line in mvect:
                mvect_list.append(float(line))
            corr = np.corrcoef(mvect_list, gvect_list)[0,1] 
            corr = corr ** 2
            #print np.corrcoef(mvect_list, gvect_list)[0,1]            
            population[i].errors(float(g.read()), corr)
            #print population[i].getErrors()
            cat_list_2[0][j], corr_list_2[0][j] = population[i].getErrors()
        #print cat_list_2, corr_list_2
        cat_list[0][i] = sum(cat_list_2[0][:])/4.0
        corr_list[0][i] = sum(corr_list_2[0][:])/4.0
    #print cat_list, corr_list
    return cat_list, corr_list
    
def dominate1(function1, function2, population):
    bool_list = []
    for i in range(0, function1.shape[1]):
        for j in range(0, function2.shape[1]):
            if i != j:
                if (function1[0][i] < function1[0][j]) and (function2[0][i] > function2[0][j]):
                    #del population[j]
                    bool_list.append('T')
                    population[j].bool = 'T'
                    break
                else:
                    continue
            else:
                #population[i].Copy()
                bool_list.append('F')
                break
                
    for individual in population:
        if individual.bool =='T':
            population.remove(individual)

    print population
    print 
    #print bool_list
    return population
    
def dominate2(function1, function2, population):
    bool_list = []
    for i in range(0, len(function1)):
        for j in range(0, len(function2)):
            if i != j:
                if (function1[i] < function1[j]) and (function2[i] > function2[j]):
                    #del population[j]
                    bool_list.append('T')
                    population[j].bool = 'T'
                    break
                elif function1[i] == function1[j] and function2[i] == function2[j]:
                    rand = np.random.random()
                    if rand > 0.5:
                        population[j].bool = 'T'
                else:
                    continue
            else:
                #population[i].Copy()
                bool_list.append('F')
                break
                
    for individual in population:
        if individual.bool =='T':
            population.remove(individual)

    #print population
    print 
    print len(population)
    print
    print bool_list
    print
    
    return population
    
def run_second(population):
    cat_list = []
    corr_list = []
    for i in range(0, len(population)):
        parent = population[i].getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        cat_list_2 = MatrixCreate2(1, 4)
        corr_list_2 = MatrixCreate2(1, 4)
        for j in range(0, 4):
            runner(j)
            g = open('categorize.csv', 'r')
            gvect = open('g_vector.txt', 'r')
            gvect_list = []
            for line in gvect:
                gvect_list.append(float(line))
    
            mvect = open('m_vector.txt', 'r')
            mvect_list = []
            for line in mvect:
                mvect_list.append(float(line))
            corr = np.corrcoef(mvect_list, gvect_list)[0,1] 
            corr = corr ** 2
            #print np.corrcoef(mvect_list, gvect_list)[0,1] 
            population[i].errors(float(g.read()), corr)
            cat_list_2[0][j], corr_list_2[0][j] = population[i].getErrors()
        cat_list.append( sum(cat_list_2[0][:])/4.0 )
        corr_list.append( sum(corr_list_2[0][:])/4.0)
        
    for i in range(0, 10 - len(population)):
        #randomNum = np.random.random()
        #if randomNum > 0.5:
        parent = np.random.choice(population)
        population.append(parent)
        parent = parent.getMutatedVersion()
        #parent = parent.getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        cat_list_2 = MatrixCreate2(1, 4)
        corr_list_2 = MatrixCreate2(1, 4)
        for j in range(0, 4):
            runner(j)
            g = open('categorize.csv', 'r')
            gvect = open('g_vector.txt', 'r')
            gvect_list = []
            for line in gvect:
                gvect_list.append(float(line))
    
            mvect = open('m_vector.txt', 'r')
            mvect_list = []
            for line in mvect:
                mvect_list.append(float(line))
            corr = np.corrcoef(mvect_list, gvect_list)[0,1] 
            #print np.corrcoef(mvect_list, gvect_list)[0,1] 
            population[i].errors(float(g.read()), corr)
            cat_list_2[0][j], corr_list_2[0][j] = population[i].getErrors()
        cat_list.append(sum(cat_list_2[0][:])/4.0)
        corr_list.append(sum(corr_list_2[0][:])/4.0)
        #else:
            #continue
    return cat_list, corr_list
    
def initial(POP_SIZE):
    oldpopulation = [Individual() for i in range(0, POP_SIZE)]
    cat, corr = run(oldpopulation)
    newpopulation = dominate1(cat, corr, oldpopulation)
    return newpopulation
    
def secondary(newpopulation):
    cat, corr = run_second(newpopulation)
    newpopulation = dominate2(cat, corr, newpopulation)
    return newpopulation
    
def iterations(generations, POP_SIZE):
    for i in range(0, generations):
        if i == 0:
            newpopulation = initial(POP_SIZE)
        if i > 0:
            newpopulation = secondary(newpopulation)
            
    return newpopulation

def dominate1_lazy(function1, population):
    bool_list = []
    for i in range(0, function1.shape[1]):
        for j in range(0, function1.shape[1]):
            if i != j:
                if (function1[0][i] < function1[0][j]):
                    #del population[j]
                    bool_list.append('T')
                    population[j].bool = 'T'
                    break
                else:
                    continue
            else:
                #population[i].Copy()
                bool_list.append('F')
                break
                
    for individual in population:
        if individual.bool =='T':
            population.remove(individual)

    print len(population)
    print 
    print bool_list
    return population
    
def dominate2_lazy(function1, population):
    bool_list = []
    for i in range(0, len(function1)):
        for j in range(0, len(function1)):
            if i != j:
                if (function1[i] < function1[j]):
                    #del population[j]
                    bool_list.append('T')
                    population[j].bool = 'T'
                    break
                elif function1[i] == function1[j]:
                    rand = np.random.random()
                    if rand > 0.5:
                        population[j].bool = 'T'
                else:
                    continue
            else:
                #population[i].Copy()
                bool_list.append('F')
                break
                
    for individual in population:
        if individual.bool =='T':
            population.remove(individual)

    #print population
    print 
    print len(population)
    print
    print bool_list
    print
    
    return population
    
def initial_lazy(POP_SIZE):
    oldpopulation = [Individual() for i in range(0, POP_SIZE)]
    cat, corr = run(oldpopulation)
    newpopulation = dominate1_lazy(cat, corr, oldpopulation)
    return newpopulation
    
def secondary_lazy(newpopulation):
    cat, corr = run_second(newpopulation)
    newpopulation = dominate2_lazy(cat, corr, newpopulation)
    return newpopulation
    
def iterations_lazy(generations, POP_SIZE):
    for i in range(0, generations):
        if i == 0:
            newpopulation = initial_lazy(POP_SIZE)
        if i > 0:
            newpopulation = secondary_lazy(newpopulation)
            
    return newpopulation






newpop = iterations(10, POP_SIZE)

