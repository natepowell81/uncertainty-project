import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math as m
import os.path
import sys
import time
import csv
import pickle


def MatrixCreate2(size, generations):
    return np.zeros((size, generations), dtype=float)

def MatrixRandomize(v):      #Creates Random Matrix
    rows = len(v)
    columns = len(v[0])
    for i in range(rows):
        for j in range(columns):
            v[i][j] = np.random.uniform(-1, 1)
    return v


def Send_Values_ToFile(data, filename):
    with open(filename, 'wb') as csvfile:
        testwriter = csv.writer(csvfile, delimiter = ',')
        testwriter.writerows(data)


parent = MatrixCreate2(4, 26)
parent = MatrixRandomize(parent)

Send_Values_ToFile(parent, "weights_Matrix.csv")