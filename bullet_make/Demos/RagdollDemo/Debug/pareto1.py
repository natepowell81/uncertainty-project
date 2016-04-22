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


def pos_size_fit(parent):
    """Finds the C and P values for the objective function"""
    Send_Values_ToFile(parent, "weights_Matrix.csv")
    cat_fit = [0.0, 0.0, 0.0, 0.0]
    pred_fit = [0.0, 0.0, 0.0, 0.0]
    for i in range(0,4):
        runner(i)
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
    return cat, pred

def vectors(p, cat, pred, parent, cat_vector, pred_vector):
    cat_vector[0][p] = cat
    pred_vector[0][p] = pred
    return cat_vector, pred_vector

def evolve(rows, columns, prob, generations):
    delta = 0
    EvoA = {}  #Address this ASAP
    weights_dict = {}
    #dominat = {}
    pred_vector = MatrixCreate2(1, 10)   #Predict Values
    cat_vector = MatrixCreate2(1, 10)   #Categorize values
    for g in range(0, generations+1):
        if g == 1:
            for p in range(0, 10):
                parent = MatrixCreate2(rows, columns)
                parent = MatrixRandomize(parent)
                parent_cat, parent_pred = pos_size_fit(parent)
                weights_dict[p] = parent
                cat_vector, pred_vector = vectors(p, parent_cat, parent_pred, parent, cat_vector, pred_vector)
        #print len(cat_vector)
        #print len(pred_vector)

        if g > 1:
            for k in range(0, 10):
                for key in nondominated.keys():
                    if k == key:
                        parent = nondominated[key]
                else:
                    temp = np.random.choice(nondominated.keys(), 1, replace=True)
                    temp = int(temp)
                    parent = nondominated[temp]
                    parent = matrixPerturb(parent, prob)

                weights_dict[k] = parent
                parent_cat, parent_pred = pos_size_fit(parent)
                cat_vector, pred_vector = vectors(k, parent_cat, parent_pred, parent, cat_vector, pred_vector)

        nondominated = {}
        for i in range(0, 10):
            for j in range(0, 10):
                if i != j:
                    if cat_vector[0][i] < cat_vector[0][j] and pred_vector[0][i] < pred_vector[0][j]:
                        if cat_vector[0][i] == cat_vector[0][j] and pred_vector[0][i] == pred_vector[0][j]:
                            nondominated[i] = weights_dict[i]
                            #dominated[j] = weights_dict[j]

                            #if dominated[j] == nondominated[i]:
                             #   print "Error: Dominated and Nondominated are equal (python code)"

        #del dominated

        if len(weights_dict.keys()) > 10:
            print "Something is wrong with the Weights Dictionary in Python (out of range)"
        if len(nondominated.keys()) > 10:
            print "Something is wrong with the nondominated dictionary in Python (out of range)"
    return


evolve(4, 13, 0.05, 50)
