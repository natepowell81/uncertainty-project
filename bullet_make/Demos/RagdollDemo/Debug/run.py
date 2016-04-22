import os
import numpy as np
import CnM_bot
import CM_bot
import CR_bot
import cat_bot
import matplotlib.pyplot as plt
import scipy.stats
import random
from savefig import save
import datetime
import pickle

def save_obj(obj, name ):
    with open('obj/'+ name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name ):
    with open('obj/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)


timesteps = 30
POP_SIZE = 20
generations = 100

population_dict = {}
population_dict['CnM'] = {}
population_dict['CM'] = {}
population_dict['CR'] = {}
population_dict['C'] = {}


num_runs = 10

if __name__ == '__main__':
    
    for i in range(0,num_runs):
        random.seed(i)
        
        population_dict['CnM'][i] = CnM_bot.iterations_CnM(generations, POP_SIZE, timesteps)
    
        population_dict['CM'][i] = CM_bot.iterations_CM(generations, POP_SIZE, timesteps)
        
        population_dict['CR'][i] = CR_bot.iterations_CR(generations, POP_SIZE, timesteps)
        
        population_dict['C'][i] = cat_bot.iterations_C(generations, POP_SIZE, timesteps)


#save_obj(population_dict, 'dictionary_values'+datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M:%S'))

CnM_test_list = []
CM_test_list = []
CR_test_list = []
C_test_list = []
for j in range(0, num_runs):
    CnM_test_list.append( population_dict['CnM'][j][len(population_dict['CnM'][j])-1].categorize )
    
    CM_test_list.append( population_dict['CM'][j][len(population_dict['CM'][j])-1].categorize )
    
    CR_test_list.append( population_dict['CR'][j][len(population_dict['CR'][j])-1].categorize )
    
    C_test_list.append( population_dict['C'][j][len(population_dict['C'][j])-1].categorize )

    

    
means_list = [sum(C_test_list)/float(num_runs), sum(CnM_test_list)/float(num_runs), sum(CM_test_list)/float(num_runs),sum(CR_test_list)/float(num_runs)]
objects = ('C', 'CnM', 'CM', 'CR')
y_pos = np.arange(len(objects))

std_err_C = np.std(C_test_list)
std_err_CnM = np.std(CnM_test_list)
std_err_CM = np.std(CM_test_list)
std_err_CR = np.std(CR_test_list)

errors = [std_err_C, std_err_CnM, std_err_CM, std_err_CR]

plt.bar(y_pos, means_list, align='center', width=0.7, yerr=errors, alpha=0.6 )
plt.xlabel('Condition for Evolution')
plt.xticks(y_pos, objects)
plt.ylabel('Average Error for Categorization')
plt.ylim([0, max(means_list)+0.1])
plt.title('Average Error')

save('/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/Images/Means_werr'+datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M:%S'))

CnM_CM_t, CnM_CM_p= scipy.stats.ttest_ind(CnM_test_list, CM_test_list)
CnM_CR_t, CnM_CR_p= scipy.stats.ttest_ind(CnM_test_list, CR_test_list) 
CM_CR_t, CM_CR_p= scipy.stats.ttest_ind(CM_test_list, CR_test_list)    #need to test C

import pandas as pd

df = pd.DataFrame(columns = ['ID', 'C', 'CnM', 'CM', 'CR'])
df['ID'] = range(0,10)
df['C'] = C_test_list
df['CnM'] = CnM_test_list
df['CM'] = CM_test_list
df['CR'] = CR_test_list
df.to_csv('Images/means_err_data2.csv', sep='\t')

#from statsmodels.stats.multicomp import (pairwise_tukeyhsd, MultiComparison)
#mod = pairwise_tukeyhsd(df['CnM'], df['CM'], df['CR'])
#print mod[0]

    
#CM_bot.Send_Values_ToFile([means_list, errors], 'Images/CR/Individuals_'+datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d'))
