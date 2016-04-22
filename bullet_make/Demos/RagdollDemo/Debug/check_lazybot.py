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
global ID 
POP_SIZE = 10
BIG_NUMBER = 999999999

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


class Individual:
    
    uid = 0
    
    def __init__(self):
        Individual.uid += 1
        self.uid = Individual.uid
        self.categorizationError = BIG_NUMBER
        self.predictionError = BIG_NUMBER
        self.ANN = MatrixCreate2(NUM_INPUTS_ANN, NUM_OUTPUTS_ANN)
        self.random = MatrixRandomize(self.ANN)        
        self.bool = 'F' #nondominated
        self.age = 0    #initial age
        self.fit = 0.0  #initial fitness
        
    def Copy(self):
        newIndividual = MatrixCreate2(NUM_INPUTS_ANN, NUM_OUTPUTS_ANN)
        for rowidx in range(0, NUM_INPUTS_ANN):
            for colidx in range(0, NUM_OUTPUTS_ANN):
                newIndividual[rowidx][colidx] = self.ANN[rowidx][colidx]
        return newIndividual

    def getMutatedVersion(self):
        self.ANN = matrixPerturb(self.ANN, 0.05)
        return self

        
    def getValues(self):
        return self.ANN
    
    def errors(self, cat_error, pred_error):
        self.categorizationError = cat_error
        self.predictionError = pred_error
        return 
        
    def getFit(self):
        return self.fit

oldpopulation = [Individual() for i in range(0,10)]

for individual in oldpopulation:
    for individual2 in oldpopulation:
        if individual == individual2:
            print 'works'
        


id_list = [oldpopulation[i].uid for i in range(0, len(oldpopulation))]


