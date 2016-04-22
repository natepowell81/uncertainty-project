import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import datetime
from savefig import save
from os import listdir
from os.path import isfile, join
import scipy.stats
import statsmodels.formula.api as sm
ind = 3841

filepath = '/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/Images/CR/Individuals_2016-01-27/Individuals23149_data_for_neurons/'

onlyfiles = [f for f in listdir(filepath) if isfile(join(filepath, f))]

position_list = [[0.0, 0.0, 6.2, 1.0, 1.0, -0.5, 'blind', 'Small'], [0.0, 0.0, 6.2, 1.5, 1.5, 0.5, 'blind', 'Large'],
                    [1.5, 0.0, 5.5, 1.0, 1.0, -0.5, 'visible', 'Small'], [1.5, 0.0, 5.5, 1.5, 1.5, 0.5, 'visible', 'Large'],
                    [3.5, 0.0, 5.5, 1.5, 1.5, 0.5, 'blind', 'Large'], [3.5, 0.0, 5.5, 1.0, 1.0, -0.5, 'blind', 'Small'],
                    [5.7, 0.0, 4.2, 1.5, 1.5, 0.5, 'visible', 'Large'], [5.7, 0.0, 4.2, 1.0, 1.0, -0.5, 'visible', 'Small'],
                    [-1.5, 0.0, 5.0, 1.5, 1.5, 0.5, 'visible', 'Large'], [-1.5, 0.0, 5.0, 1.0, 1.0, -0.5, 'visible', 'Small'],
                    [-3.5, 0.0, 5.0, 1.5, 1.5, 0.5, 'blind', 'Large'], [-3.5, 0.0, 5.0, 1.0, 1.0, -0.5, 'blind', 'Small'],
                    [-5.7, 0.0, 4.2, 1.5, 1.5, 0.5, 'visible', 'Large'], [-5.7, 0.0, 4.2, 1.0, 1.0, -0.5, 'visible', 'Small']]


lin_regression = {}

for filename in onlyfiles:
    
    data1 = pd.read_csv(filepath+filename, header=0, delim_whitespace=False)
    data2 = pd.DataFrame(columns=['c', 'm', 'g_var'])

    data2.loc[0, 'c'] = data1.columns.values[0]
    data2.loc[0, 'm'] = data1.columns.values[1]
    data2.loc[0, 'g_var'] = data1.columns.values[2]
    for i in range(1, data1.shape[0]+1):
        data2.loc[i, 'c'] = data1.loc[i-1, data1.columns.values[0]]
        data2.loc[i, 'm'] = data1.loc[i-1, data1.columns.values[1]]
        data2.loc[i, 'g_var'] = data1.loc[i-1, data1.columns.values[2]]
    
    #results = sm.OLS(np.asarray(data2['c']), np.asarray(data2['g_var'])).fit()
    #results.summary()
    index = 0
    
    #lin_regression[index] = results
    #lin_regression[index]['r'] = r
    #lin_regression[index]['p'] = p 
    index = index + 1
    timestep_list = range(0,data2.shape[0])
        
    f, ax = plt.subplots(3, sharex=True)
        
    ax[0].plot(timestep_list, data2['c'], color='r')
    ax[1].plot(timestep_list, data2['m'], color='b')
    ax[2].plot(timestep_list, data2['g_var'], color='g')
    #ax[0].set_ylim([0, float(max(data2['c']))])
    #ax[1].set_ylim([float(min(data2['m'])), float(max(data2['m']))])
    #ax[2].set_ylim([0,1.3])
    ax[2].set_xlim([0,max(timestep_list)])
    #ax[0].legend(['categorize'])
    #ax[1].legend(['movement'])
    #ax[2].legend(['uncertainty'])
    ax[0].set_ylabel('Error')
    ax[1].set_ylabel('Movement')
    ax[2].set_ylabel('Uncertainty')
    ax[2].set_xlabel('Number of Timesteps')
    ax[0].set_title('Performance of Individual')
    plt.show()


        
        
import os.path
import csv

def Send_Values_ToFile(data, filename):
    with open(filename, 'wb') as csvfile:
        testwriter = csv.writer(csvfile, delimiter = ',')
        testwriter.writerows(data)

def Send_Values_ToFile_two(data, filename):
    with open(filename, "wb") as output:
        writer = csv.writer(output, lineterminator=',')
        for val in data:
            writer.writerow([val])

import pandas as pd
ANN = pd.read_csv('/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/Images/CR/Individuals_2016-01-27/Individuals23149_data_for_neurons/723149blindLarge_ANN.txt', header=None, delimiter=',')

first_list = []
for i in ANN.columns.values:
    first_list.append(float(i))
ANN_matrix = list(ANN.as_matrix())

#parent = ANN_matrix
filename = "weights_Matrix.csv"
Send_Values_ToFile(ANN_matrix, filename)
Send_Values_ToFile_two(position_list[7], "position_size.csv")
os.system('./AppRagdollDemo 10')

        
        
        
    #print data1