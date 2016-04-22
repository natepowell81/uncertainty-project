import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math as m
import os.path
import sys
import time
import csv

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

#all_weights = MatrixCreate2(17,17)
"""The first 4 rows are weights between proprieceptive sensors and everything
    The second 4 rows are between hidden layer and everything
    The third 4 rows are between the prediction neurons
    The fourth 4 rows are for the motor neurons
    And the last row is for the guess neurons """

#all_weights = MatrixRandomize(all_weights)

# OR

all_weights = MatrixRandomize(MatrixCreate2(4, 13))
"""This matrix is different from the one above because it only has
the weights that we are interested in, except the guess neuron"""


#H = MatrixCreate2(1,4) #Hidden Neurons, there are four of them for now.

#P = MatrixCreate2(1,4) #This is simulated data for now, this will eventually come from the C++ file
                        # proprieceptive neurons giving the angle of the joint
#P = MatrixRandomize(P)

#P_hat = MatrixCreate2(4, 4) #Prediction Neurons of the Proprieceptive, 4 from each hidden neuron

#Motor_Neurons = MatrixCreate2(1, 4)

#for h in range(0,4):     #To Find Hidden Neuron Values
#    H[0][h] = 0.0
#    for p in range(0,4):
#        H[0][h] = H[0][h] + all_weights[p][4+h] * P[0][p]

#for h in range(0, 4):     #To Find Predictive Neuron Values
#    for p in range(0,4):
#        P_hat[h][p] = P_hat[h][p] + H[0][h] * all_weights[h+4][p+8]

#for h in range(0, 4):     #To Find the Motor Neuron Values
#    for p in range(0, 4):
#        Motor_Neurons[0][h] = Motor_Neurons[0][h] + H[0][h] * all_weights[h+4][p+12]

Send_Values_ToFile(all_weights, "weights_Matrix.csv")
#Send_Values_ToFile(H, "hidden_Neurons.csv")
#Send_Values_ToFile(P_hat, "prediction_Neurons.csv")
#Send_Values_ToFile(Motor_Neurons, "motor_Neurons.csv")

def evolve(rows, columns, prob, generations):
    parent = MatrixCreate2(rows, columns)
    #print ("parent", parent)
    parent = MatrixRandomize(parent)
    #print ("parentRandomized", parent)
    parentFitness = pos_size_fit(parent)
    #print ('parent fitness', parentFitness)
    #NeuronValues = MatrixCreate2(rows, columns)
    #NeuronValues[0] = 0.5
    fits = MatrixCreate(generations)
    for currentGeneration in range(0, generations):
        #print currentGeneration, parentFitness
        child = matrixPerturb(parent, prob)
        childFitness = pos_size_fit(child)
        if ( childFitness < parentFitness ):
            parent = child
            parentFitness = childFitness
            fits[currentGeneration] = parentFitness
        #print parentFitness, childFitness
        print currentGeneration, parentFitness, childFitness
    return

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


def categorizer(i):
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

def pos_size_fit(parent):
    Send_Values_ToFile(parent, "weights_Matrix.csv")
    cat_fit = [0.0, 0.0, 0.0, 0.0]
    pred_fit = [0.0, 0.0, 0.0, 0.0]
    for i in range(0,4):
        categorizer(i)
        f = open('categorize.csv', 'r')
        p = open('predict.csv', 'r')
        cat_fit[i] = float(f.read().rstrip().split(',')[0])
        pred_fit[i] = float(p.read().rstrip().split(',')[0])
    cat = sum(cat_fit) / 4.0
    pred = sum(pred_fit) / 4.0
    #print cat_fit
    #print pred_fit
    print "cat/pred =", cat, pred
    if cat*pred > 9.00:
        print "Something is wrong with the Fitness Function. Fix it."
    return cat*pred






evolve(4,13, 0.05, 50)
