import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math as m
import os.path
import sys
import time
#from synapse import *

numSensors = 4;
numMotors = 8;
numGenerations = 1000;

def MatrixCreate(size):  #Creates Matrix of zeros
    return np.zeros((size), dtype=float)

def MatrixCreate2(size, generations):
    return np.zeros((size, generations), dtype=float)

v = MatrixCreate(10)

def MatrixRandomize(v):      #Creates Random Matrix
    rows = len(v)
    columns = len(v[0])
    for i in range(rows):
        for j in range(columns):
            v[i][j] = np.random.uniform(-1, 1)
    return v

def MatrixRandomize2(v):      #Creates Random Matrix
    rows = len(v)
    for i in range(rows):
        v[i] = np.random.uniform(-1, 1)
    return v

def matrixPerturb(v, prob):
    p = np.copy(v)
    for i in range(len(v)):
        for j in range(len(v[0])):
            if prob > np.random.rand():
                p[i][j] = np.random.uniform(-1, 1);
    return p

def matrixPerturb2(v, prob):
    p = np.copy(v)
    for i in range(len(v)):
        if prob > np.random.rand():
            p[i] = np.random.uniform(-1, 1);
    return p

def PlotVectorAsLine(g):
    plt.plot(g)
    plt.ylabel('Fitness')
    plt.xlabel('Generation')

#Rows = 4 and columns = 8
def update(nv, Synapse, rows, columns):
    N = MatrixCreate2(rows, columns)
    for e in range(1, columns):
        #N = MatrixCreate2(1, 10)
        for j in range(rows):   # rows and columns of Synapses
            for k in range(columns):
                N[k] = (nv[e - 1, k] * Synapse[j][k])
            #print ("N", N)
            temp = sum(N)
            #print temp
            if temp > 1:
                nv[e, j] = 1
            elif temp < 0:
                nv[e, j] = 0
            else:
                nv[e, j] = temp
    return nv

def MeanDistance(v1, v2):   # Note that V1 and V2 have to be the same length
    d = MatrixCreate(len(v1))  #Also Is finding the fitness because of 1 - meandistance
    for i in range(len(v1)):     #They are vectors
        d[i] = ((v1[i] - v2[i]) ** 2)
    return 1 - (sum(d) / len(v1))


def fitness(v):
    nv = MatrixCreate2(10, 10) #NumberUpdates x NumberNeurons
    nv[0, :] = 0.5
    G = update(nv, v, 10)
    #print ('updated neuronvalues', G)
    aNV = nv[9,:]           #actual Neuron Values
    dNV = MatrixCreate(10)  #desired Neuron Values
    for j in range(0,10,2):
        dNV[j]=1
    #print ('dNV', dNV)
    B = MeanDistance(aNV, dNV)   #Uses Mean Square Error
    #print ('B', B)
    return 1 - B

def fitness2(parent, rows, columns):
    nv = MatrixCreate2(rows, columns) #NumberUpdates x NumberNeurons
    nv[0, :] = 0.5
    G = update(nv, v, rows, columns)
    #print ('updated neuronvalues', G)
    aNV = nv[9,:]           #actual Neuron Values
    dNV = MatrixCreate(columns)  #desired Neuron Values
    for j in range(0,10,2):
        dNV[j]=1
    #print ('dNV', dNV)
    diff=0.0
    for i in range(1,columns):
        for e in range(rows,columns):
            diff=diff + abs(nv[i,e]-nv[i,e+1])
            diff=diff + abs(nv[i+1,e]-nv[i,e])
    diff=diff/(2*8*9)
    #print ('B', B)
    return diff


def evolve(rows, columns, prob, generations):
    parent = MatrixCreate2(rows, columns)
    #print ("parent", parent)
    parent = MatrixRandomize(parent)
    #print ("parentRandomized", parent)
    parentFitness = Fitness3_Get(parent)
    #print ('parent fitness', parentFitness)
    NeuronValues = MatrixCreate2(rows, columns)
    NeuronValues[0] = 0.5
    fits = MatrixCreate(generations)
    for currentGeneration in range(0,1000):
        #print currentGeneration, parentFitness
        child = matrixPerturb(parent, prob)
        childFitness = Fitness3_Get(child)
        if ( childFitness > parentFitness ):
            parent = child
            parentFitness = childFitness
            fits[currentGeneration] = parentFitness
        #print parentFitness, childFitness
        print currentGeneration, parentFitness, childFitness
    return 



############################################################
import csv


Synapse = MatrixCreate2(numSensors, numMotors)
for e in range(numSensors):
    for h in range(numMotors):
        Synapse[e][h] = np.random.uniform(-1, 1)




def Send_Synapse_Weights_ToFile(data, filename):
    with open(filename, 'wb') as csvfile:
        testwriter = csv.writer(csvfile, delimiter = ',')
        testwriter.writerows(data)


def Fitness_Collect_From_File(filename):
    fits = []
    with open(filename, 'rb') as csvfile:
        testreader = csv.reader(csvfile, delimiter = ',')
        for i in testreader:
            fits.append(testreader[i])

        return testreader[0]

#def Fitness3_Get(Synapses):
#    weightsFileName = "weights.csv"

#    fitFileName = "fits.csv"
#    Send_Synapse_Weights_ToFile(Synapses,weightsFileName)
#    os.system('./AppRagdollDemo');
#    while not os.path.isfile(fitFileName):
#        time.sleep(0.5);
#
#    fitness = Fitness_Collect_From_File(fitFileName)
#    Simulate_Robot()
#    #Wait_For_Fitness_File(fitFileName)
#    #Delete_File(weightsFileName)
#    #Delete_File(fitFileName)
#    #return( fitness )
#    pass



def Fitness3_Get(parent):
    f = open('weights.csv','w')     # f = file of weights
    for i in range(len(parent)):
        f.write('{0:.3f}'.format(parent[i][0]))
        for j in range(1,len(parent[i])):
            f.write(',{0:.3f}'.format(parent[i][j]))
        f.write('\n')
    f.close()
    if os.path.isfile('distance.csv'):
        os.remove('distance.csv')
    os.system('./AppRagdollDemo')
    while not os.path.isfile('fits.csv'):
        time.sleep(0.5)
    f = open('fits.csv','r')
    zdis = f.read().rstrip().split(',')[2]
    zdis = float(zdis)
    return zdis
################################################################

#Synapse = Synapse(numSensors, numMotors)
#print (Synapse, "Synapse")

#Fitness3_Get(Synapse)





#NeuronValues = MatrixCreate2(rows, columns)
#NeuronValues[0] = 0.5

#parent = MatrixCreate2(rows, columns)
#print ("parent", parent)
#parent = MatrixRandomize(parent)
#print ("parentRandomized", parent)
#parentFitness = fitness2(parent)
#print ('parent fitness', parentFitness)
#NeuronValues = MatrixCreate2(rows, columns)
#NeuronValues[0] = 0.5
#fits = MatrixCreate(generations)
#for currentGeneration in range(0,1000):
    #print currentGeneration, parentFitness
    #child = matrixPerturb(parent, 0.05)
    #childFitness = fitness2(child)
    #if ( childFitness > parentFitness ):
        #parent = child
        #parentFitness = childFitness
    #print parentFitness, childFitness
    #fits[currentGeneration] = parentFitness

fits = evolve(numSensors, numMotors, 0.05, numGenerations)
#updated = update(NeuronValues, parent, 10)
#print ('updated after', updated)

#plt.imshow(updated, cmap=cm.gray, aspect='auto',interpolation='nearest')

#plt.show()
#PlotVectorAsLine(fits)
#plt.show()
