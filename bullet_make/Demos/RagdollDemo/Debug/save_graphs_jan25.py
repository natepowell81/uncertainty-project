import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import datetime
from savefig import save


data1 = pd.read_csv('Images/means_err_dataJan27.csv', header=False, delim_whitespace=True)

CnM_test_list = data1['CnM']
CM_test_list = data1['CM']
CR_test_list = data1['CR']
C_test_list = data1['C']

num_runs = 15

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

save('/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/Images/Means_werr'+datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M:%S')+'Jan27')

