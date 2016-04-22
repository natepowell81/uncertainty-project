import os
import numpy as np
import CnM_bot
import CM_bot
import CR_bot
import matplotlib.pyplot as plt
import scipy.stats
import random
from savefig import save
import datetime


timesteps = 20
POP_SIZE = 15
generations = 100

population_dict = {}
population_dict['CnM'] = {}
population_dict['CM'] = {}
population_dict['CR'] = {}


num_runs = 10
if __name__ == '__main__':
    
    for i in range(0,num_runs):
        random.seed(i)
        
        #population_dict['CnM'][i] = CnM_bot.iterations_CnM(generations, POP_SIZE, timesteps)
    
        #population_dict['CM'][i] = CM_bot.iterations_CM(generations, POP_SIZE, timesteps)
        
        population_dict['CR'][i] = CR_bot.iterations_CR(generations, POP_SIZE, timesteps)


CnM_test_list = []
CM_test_list = []
CR_test_list = []
for j in range(0, num_runs):
    #CnM_test_list.append( population_dict['CnM'][j][len(population_dict['CnM'][j])-1].categorize )
    
    #CM_test_list.append( population_dict['CM'][j][len(population_dict['CM'][j])-1].categorize )
    
    CR_test_list.append( population_dict['CR'][j][len(population_dict['CR'][j])-1].categorize )
    
CR_mean = sum(CR_test_list)/float(num_runs)
    