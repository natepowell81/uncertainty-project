import numpy as  np
import random
def runner_positions(i):
    position_list = [(0.0, 0.0, 6.2, 1.0, 1.0, -0.5), (0.0, 0.0, 6.2, 1.5, 1.5, 0.5),
                    (1.5, 0.0, 5.5, 1.0, 1.0, -0.5), (1.5, 0.0, 5.5, 1.5, 1.5, 0.5),
                    (3.5, 0.0, 5.5, 1.5, 1.5, 0.5), (3.5, 0.0, 5.5, 1.0, 1.0, -0.5),
                    (5.7, 0.0, 4.2, 1.5, 1.5, 0.5), (5.7, 0.0, 4.2, 1.0, 1.0, -0.5),
                    (-1.5, 0.0, 5.0, 1.5, 1.5, 0.5), (-1.5, 0.0, 5.0, 1.0, 1.0, -0.5),
                    (-3.5, 0.0, 5.0, 1.5, 1.5, 0.5), (-3.5, 0.0, 5.0, 1.0, 1.0, -0.5),
                    (-5.7, 0.0, 4.2, 1.5, 1.5, 0.5), (-5.7, 0.0, 4.2, 1.0, 1.0, -0.5)]

    training_pos = random.sample(position_list,7)
    
    i = xrange(len(position_list))
    test = random.sample(i, 7)

    test_index = [blah for blah in i if blah not in test]
    test_list = []
    for num in test_index:
        test_list.append(position_list[num])
        
    return training_pos, test_index
    
    
def run_training(training_list):
    for position in training_list:
        position_size = position           #Center robot should be blind to this
        Send_Values_ToFile_two(position_size, "position_size.csv")
        os.system('./AppRagdollDemo')
        
def run_testing(test_list):
    for position in test_list:
        position_size = position           #Center robot should be blind to this
        Send_Values_ToFile_two(position_size, "position_size.csv")
        os.system('./AppRagdollDemo')
        
position_list = [(0.0, 0.0, 6.2, 1.0, 1.0, -0.5), (0.0, 0.0, 6.2, 1.5, 1.5, 0.5),
                (1.5, 0.0, 5.5, 1.0, 1.0, -0.5), (1.5, 0.0, 5.5, 1.5, 1.5, 0.5),
                (3.5, 0.0, 5.5, 1.5, 1.5, 0.5), (3.5, 0.0, 5.5, 1.0, 1.0, -0.5),
                (5.7, 0.0, 4.2, 1.5, 1.5, 0.5), (5.7, 0.0, 4.2, 1.0, 1.0, -0.5),
                (-1.5, 0.0, 5.0, 1.5, 1.5, 0.5), (-1.5, 0.0, 5.0, 1.0, 1.0, -0.5),
                (-3.5, 0.0, 5.0, 1.5, 1.5, 0.5), (-3.5, 0.0, 5.0, 1.0, 1.0, -0.5),
                (-5.7, 0.0, 4.2, 1.5, 1.5, 0.5), (-5.7, 0.0, 4.2, 1.0, 1.0, -0.5)]

training_pos = random.sample(position_list,7)

i = xrange(len(position_list))
test = random.sample(i, 7)
test_index = [blah for blah in i if blah not in test]
test_list = []
for num in test_index:
    test_list.append(position_list[num])
    

for i in test_list:
     print i
    
    
    
    