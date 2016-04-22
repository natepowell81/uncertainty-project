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

C_med = median(C_test_list)
CnM_med = median(CnM_test_list)
CM_med = median(CM_test_list)
CR_med = median(CR_test_list)

q75_c, q25_c = np.percentile(C_test_list, [75 ,25])
q75_cnm, q25_cnm = np.percentile(CnM_test_list, [75 ,25])
q75_cm, q25_cm = np.percentile(CM_test_list, [75 ,25])
q75_cr, q25_cr = np.percentile(CR_test_list, [75 ,25])

num_runs = 15

f, ax = plt.subplots(2,2)


CnM_mean = round(sum(data1['CnM'])/15.0, 3)
CM_mean = round(sum(data1['CM'])/15.0, 3)
C_mean = round(sum(data1['C'])/15.0, 3)
CR_mean = round(sum(data1['CR'])/15.0, 3)

ax[0][0].hist(C_test_list, bins=5)
ax[0][1].hist(CnM_test_list, bins=5)
ax[1][0].hist(CM_test_list, bins=5)
ax[1][1].hist(CR_test_list, bins=5)
ax[0][0].set_ylim(0, 10)
ax[0][1].set_ylim(0, 10)
ax[1][0].set_ylim(0, 10)
ax[1][1].set_ylim(0, 10)
ax[0][0].set_xlim(0, 1)
ax[0][1].set_xlim(0, 1)
ax[1][0].set_xlim(0, 0.2)
ax[1][1].set_xlim(0, 0.1)
ax[0][0].set_title('C', fontsize=10)
ax[0][1].set_title('CnM', fontsize=10)
ax[1][0].set_title('CM', fontsize=10)
ax[1][1].set_title('CR', fontsize=10)
ax[0][0].text(.1, 9.0, 'Mean = ' + str(C_mean))
ax[0][1].text(.1, 9.0, 'Mean = ' + str(CnM_mean))
ax[1][0].text(.02, 9.0, 'Mean = ' + str(CM_mean))
ax[1][1].text(.01, 9.0, 'Mean = ' + str(CR_mean))
ax[0][0].set_ylabel('Frequency')
ax[0][0].set_xlabel('Error')
ax[0][1].set_ylabel('Frequency')
ax[0][1].set_xlabel('Error')
ax[1][0].set_ylabel('Frequency')
ax[1][0].set_xlabel('Error')
ax[1][1].set_ylabel('Frequency')
ax[1][1].set_xlabel('Error')
plt.suptitle('Distribution of Error per Condition', fontsize=12)
plt.tight_layout()
plt.show()