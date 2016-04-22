import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math as m

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

rows = 10
columns = 10
def update(nv, Synapse, i):
    N = MatrixCreate(10)
    for e in range(1, i):
        #N = MatrixCreate2(1, 10)
        for j in range(10):   # rows and columns of Synapses
            for k in range(10):
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
    for i in range(len(v1)):
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

def fitness2(v):
    nv = MatrixCreate2(10, 10) #NumberUpdates x NumberNeurons
    nv[0, :] = 0.5
    G = update(nv, v, 10)
    #print ('updated neuronvalues', G)
    aNV = nv[9,:]           #actual Neuron Values
    dNV = MatrixCreate(10)  #desired Neuron Values
    for j in range(0,10,2):
        dNV[j]=1
    #print ('dNV', dNV)
    diff=0.0
    for i in range(1,9):
        for e in range(0,9):
            diff=diff + abs(nv[i,e]-nv[i,e+1])
            diff=diff + abs(nv[i+1,e]-nv[i,e])
    diff=diff/(2*8*9)
    #print ('B', B)
    return diff


def hillclimber(rows, columns, prob, generations):
    parent = MatrixCreate2(rows, columns)
    print ("parent", parent)
    parent = MatrixRandomize(parent)
    print ("parentRandomized", parent)
    parentFitness = fitness2(parent)
    print ('parent fitness', parentFitness)
    #NeuronValues = MatrixCreate2(rows, columns)
    #NeuronValues[0] = 0.5
    fits = MatrixCreate(generations)
    for currentGeneration in range(0,1000):
        print currentGeneration, parentFitness
        child = matrixPerturb(parent, 0.05)
        childFitness = fitness2(child)
        if ( childFitness > parentFitness ):
            parent = child
            parentFitness = childFitness
        print parentFitness, childFitness
        fits[currentGeneration] = parentFitness
    return fits

NeuronValues = MatrixCreate2(rows, columns)
NeuronValues[0] = 0.5

parent = MatrixCreate2(rows, columns)
#print ("parent", parent)
parent = MatrixRandomize(parent)
#print ("parentRandomized", parent)
parentFitness = fitness2(parent)
#print ('parent fitness', parentFitness)
#NeuronValues = MatrixCreate2(rows, columns)
#NeuronValues[0] = 0.5
#fits = MatrixCreate(generations)
for currentGeneration in range(0,1000):
    #print currentGeneration, parentFitness
    child = matrixPerturb(parent, 0.05)
    childFitness = fitness2(child)
    if ( childFitness > parentFitness ):
        parent = child
        parentFitness = childFitness
    #print parentFitness, childFitness
    #fits[currentGeneration] = parentFitness

fits = hillclimber(10, 10, 0.05, 1000)
updated = update(NeuronValues, parent, 10)
print ('updated after', updated)

plt.imshow(updated, cmap=cm.gray, aspect='auto',interpolation='nearest')

plt.show()
PlotVectorAsLine(fits)
plt.show()
