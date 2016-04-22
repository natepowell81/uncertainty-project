import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math as m
import os.path
import sys
import time
import random
import csv
import pickle
from savefig import save
import datetime

NUM_INPUTS_ANN = 4
NUM_OUTPUTS_ANN = 26
POP_SIZE = 15
BIG_NUMBER = 999999999
timesteps = [20]
num_timesteps = timesteps[0]
global ID 

## classes, new stuff

def MatrixCreate2(size, generations):
    return np.zeros((size, generations), dtype=float)
    
def MatrixRandomize(v):      #Creates Random Matrix
    rows = len(v)
    columns = len(v[0])
    for i in range(rows):
        for j in range(columns):
            v[i][j] = np.random.uniform(-1, 1)
    return v

def matrixPerturb(v, prob):
    p = np.copy(v)
    for i in range(len(v)):
        for j in range(len(v[0])):
            if prob > np.random.rand():
                p[i][j] = np.random.uniform(-1, 1);
    return p

def Send_Values_ToFile(data, filename):
    with open(filename, 'wb') as csvfile:
        testwriter = csv.writer(csvfile, delimiter = ',')
        testwriter.writerows(data)
        
def Send_Values_ToFile_two(data, filename):
    with open(filename, "wb") as output:
        writer = csv.writer(output, lineterminator=',')
        for val in data:
            writer.writerow([val])

def positions():
    position_list = [[0.0, 0.0, 6.2, 1.0, 1.0, -0.5, 'blind', 'Small'], [0.0, 0.0, 6.2, 1.5, 1.5, 0.5, 'blind', 'Large'],
                    [1.5, 0.0, 5.5, 1.0, 1.0, -0.5, 'visible', 'Small'], [1.5, 0.0, 5.5, 1.5, 1.5, 0.5, 'visible', 'Large'],
                    [3.5, 0.0, 5.5, 1.5, 1.5, 0.5, 'blind', 'Large'], [3.5, 0.0, 5.5, 1.0, 1.0, -0.5, 'blind', 'Small'],
                    [5.7, 0.0, 4.2, 1.5, 1.5, 0.5, 'visible', 'Large'], [5.7, 0.0, 4.2, 1.0, 1.0, -0.5, 'visible', 'Small'],
                    [-1.5, 0.0, 5.0, 1.5, 1.5, 0.5, 'visible', 'Large'], [-1.5, 0.0, 5.0, 1.0, 1.0, -0.5, 'visible', 'Small'],
                    [-3.5, 0.0, 5.0, 1.5, 1.5, 0.5, 'blind', 'Large'], [-3.5, 0.0, 5.0, 1.0, 1.0, -0.5, 'blind', 'Small'],
                    [-5.7, 0.0, 4.2, 1.5, 1.5, 0.5, 'visible', 'Large'], [-5.7, 0.0, 4.2, 1.0, 1.0, -0.5, 'visible', 'Small']]


    training_pos = random.sample(position_list,7)
    
    i = xrange(len(position_list))
    test = random.sample(i, 7)

    test_index = [blah for blah in i if blah not in test]
    test_list = []
    for num in test_index:
        test_list.append(position_list[num])
        
    return training_pos, test_list

def run_training_CnM(population, num_timesteps, training_list):
    for individual in population:
        parent = individual.ANN
        #parent = parent.getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        move_list_2 = MatrixCreate2(1, 7)
        cat_list = MatrixCreate2(1,7)
        index = 0
        for position in training_list:
            position_size = position           #Center robot should be blind to this
            Send_Values_ToFile_two(position_size, "position_size.csv")
            os.system('./AppRagdollDemo')
            mvect = open('m_vector.txt', 'r')
            mvect_list = []
            for line in mvect:
                mvect_list.append(float(line))
            move = mvect_list[num_timesteps-1] #sum(mvect_list)
            move2 = sum(mvect_list)/float(num_timesteps)
            
            cat = open('g_mean_vector.txt', 'r')
            categorize = []
            for line in cat:
                categorize.append( float(line) )
            cat_fit =  (sum(categorize)/float(num_timesteps))    #

            #print 'move:', move
            #print np.corrcoef(mvect_list, gvect_list)[0,1]            
            #print population[i].getErrors()
            move_list_2[0][index] = move2
            cat_list[0][index] = cat_fit
            #print cat_list_2, corr_list_2
            #move_list[0][i] = sum(move_list_2[0][:])/4.0
            index = index + 1
        individual.fit = (1.0 / (1.0 + (sum(move_list_2[0][:])/7.0))) * (1.0 / (1.0 + (sum(cat_list[0][:])/7.0)))
    return population

def run_training_second_CnM(population, POP_SIZE, num_timesteps, training_list):
    move_list = []
    new_individuals = Fill_CnM(population, POP_SIZE)
    print "num new individuals =", len(new_individuals)
    for individual in new_individuals:
        parent = individual.ANN
        #parent = parent.getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        move_list_2 = MatrixCreate2(1,7)
        cat_list = MatrixCreate2(1,7)
        index = 0
        for position in training_list:
            position_size = position           #Center robot should be blind to this
            Send_Values_ToFile_two(position_size, "position_size.csv")
            os.system('./AppRagdollDemo')
            mvect = open('m_vector.txt', 'r')
            mvect_list = []
            for line in mvect:
                mvect_list.append(float(line))
            move = mvect_list[num_timesteps-1] #sum(mvect_list)
            move2 = sum(mvect_list)/float(num_timesteps)
            
            cat = open('g_mean_vector.txt', 'r')
            categorize = []
            for line in cat:
                categorize.append( float(line) )
            cat_fit =  (sum(categorize)/float(num_timesteps))    #

            #print 'move:', move
            #print np.corrcoef(mvect_list, gvect_list)[0,1]            
            #print population[i].getErrors()
            move_list_2[0][index] = move2
            cat_list[0][index] = cat_fit
            #print cat_list_2, corr_list_2
            #move_list[0][i] = sum(move_list_2[0][:])/4.0
            index = index + 1
        individual.fit = (1.0 / (1.0 + (sum(move_list_2[0][:])/7.0))) * (1.0 / (1.0 + (sum(cat_list[0][:])/7.0)))
    population = population + new_individuals
    return population
            
                    
def run_testing_CnM(test_list, population, timestep):
    timestep_list = []
    for i in range(0, timestep):
        timestep_list.append(i)
    
    individual = population[len(population)-1]
    parent = individual.ANN

    newpath = r'/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/Images/CnM/Individuals_'+datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d')
    if not os.path.exists(newpath):
        os.makedirs(newpath)
        
    newpath = r'/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/Images/CnM/Individuals_'+datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d')+'/Individuals'+str(individual.uid)+'_data_for_neurons'
    if not os.path.exists(newpath):
        os.makedirs(newpath)
    
    print 'length of Test Individuals: ', len(test_list)
    
    #parent = parent.getValues()
    filename = "weights_Matrix.csv"
    Send_Values_ToFile(parent, filename)
    index = 0
    for position in test_list:
        values_save = MatrixCreate2(timestep, 2)
        position_size = position
        #print position_size
        Send_Values_ToFile_two(position_size, "position_size.csv")
        
        os.system('./AppRagdollDemo')
        cat = open('g_mean_vector.txt', 'r')
        categorize = []
        for line in cat:
            categorize.append( float(line) )
        cat_fit = (sum(categorize)/float(num_timesteps))    #
        
        individual.categorize.append(cat_fit)
        
        mvect = open('m_vector.txt', 'r')
        mvect_list = []
        for line in mvect:
            mvect_list.append(float(line))
        
        individual.guess_neurons[index] = categorize
        
        index2 = 0
        for value in categorize[:timestep]:
            values_save[index2][0] = value
            index2 = index2 + 1
            
        index3 = 0    
        for value in mvect_list[:timestep]:
            values_save[index3][1] = value
            index3 = index3 + 1
        
        Send_Values_ToFile(values_save, 'Images/CnM/Individuals_'+datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d')+'/Individuals'+str(individual.uid)+'_data_for_neurons/'+str(index)+str(individual.uid)+position[6]+position[7]+'Categorizeandmovement.txt')

        #print 'move:', move
        #print np.corrcoef(mvect_list, gvect_list)[0,1] 
        if position[5] == 0.5:
            #if categorize[timestep-1] > 0.40 and categorize[timestep-1] < 0.60:
            if sum(categorize)/float(timestep) > 0.40 and sum(categorize)/float(timestep) < 0.60:
                individual.test = individual.test + 1 #/ float(num_timesteps)
                
        if position[5] == -0.5:
            #if categorize[timestep-1] > -0.40 and categorize[timestep-1] < -0.60:
            if sum(categorize)/float(timestep) < -0.40 and sum(categorize)/float(timestep) > -0.60:
                individual.test = individual.test + 1 #/ float(num_timesteps)
        
        #plt.hold(True)
        f, ax = plt.subplots(2, sharex=True)
        
        ax[0].plot(timestep_list, categorize[:timestep], color='r')
        ax[1].plot(timestep_list, mvect_list[:timestep], color='b')
        ax[0].set_ylim([-1,1])
        ax[1].set_ylim([min(mvect_list), max(mvect_list)])
        ax[1].set_xlim([0,timestep])
        ax[0].legend(['categorize'])
        ax[1].legend(['movement'])
        ax[0].set_ylabel('Average of Guess Neurons')
        ax[1].set_ylabel('Change in Prop Angles')
        ax[1].set_xlabel('Number of Timesteps')
        ax[0].set_title('Categorize and ~Movement Individual: '+str(individual.uid)+' '+'_Pos: '+position[6]+'&'+position[7])
        
        #save('/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/Images/CnM/Individuals_'+datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d')+'/Individuals'+str(individual.uid)+'_data_for_neurons/'+str(index)+'CnM_image_'+str(individual.uid)+'_'+datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d'))
        
        index = index + 1
    Send_Values_ToFile(parent, 'Images/CnM/Individuals_'+datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d')+'/Individuals'+str(individual.uid)+'_data_for_neurons/'+str(index)+str(individual.uid)+position[6]+position[7]+'_ANN.txt')

    individual.categorize = sum( individual.categorize ) / 7.0
    print individual.categorize   # Average 
    
    return population

filename = "weights_Matrix.csv"

class Individual:
    uid = 0
    def __init__(self):
        Individual.uid += 1
        self.uid = Individual.uid
        self.categorizationError = BIG_NUMBER
        self.predictionError = BIG_NUMBER
        self.ANN = MatrixCreate2(NUM_INPUTS_ANN, NUM_OUTPUTS_ANN)
        self.random = MatrixRandomize(self.ANN)        
        self.bool = 'F' #nondominated
        self.age = 0    #initial age
        self.fit = 0.0  #initial fitness
        self.test = 0
        self.guess_neurons = {}
        self.categorize = []
        
    def age_(self):
        self.age += 1
        return self
        
    def Copy(self):
        newIndividual = MatrixCreate2(NUM_INPUTS_ANN, NUM_OUTPUTS_ANN)
        for rowidx in range(0, NUM_INPUTS_ANN):
            for colidx in range(0, NUM_OUTPUTS_ANN):
                newIndividual[rowidx][colidx] = self.ANN[rowidx][colidx]
        return newIndividual

    def getMutatedVersion(self):
        self.ANN = matrixPerturb(self.ANN, 0.25)
        return self

        
    def getValues(self):
        return self.ANN
    
    def errors(self, cat_error, pred_error):
        self.categorizationError = cat_error
        self.predictionError = pred_error
        return 
        
    def getFit(self):
        return self.fit

oldpopulation = [Individual() for i in range(0, POP_SIZE)]

def sort_by_dom_CnM(population):
    pop_list = []
    for individual in population:
        pop_list.append((individual.bool, individual))
    pop_list.sort()
    pop = []
    for ind in pop_list:
        pop.append(ind[1])
    return pop
    
def sort_by_fit_CnM(population):
    fit_list = []
    for individual in population:
        fit_list.append((individual.fit, individual))
    fit_list.sort()
    pop = []
    for ind in fit_list:
        pop.append(ind[1])
    return pop

def AGE_individuals_CnM(population):
    for individual in population:
        individual = individual.age_()
    return population
        
def Inject_CnM(population):
    population.append(Individual())
    return population        
    
def Fill_CnM(population, POP_SIZE):
    new_individuals = []
    for i in range(0, POP_SIZE - len(population) - 1):
        individual = np.random.choice(population, 1, replace=True)
        individual2 = Individual()
        individual2.ANN = individual[0].ANN
        individual2 = individual2.getMutatedVersion()
        new_individuals.append(individual2)
    new_individuals = Inject_CnM(new_individuals)
    return new_individuals

def del_doms_CnM(population):
    #print [population[i].bool for i in range(0, len(population))]
    population = [individual for individual in population if individual.bool != 'T']
    #print [population[i].bool for i in range(0, len(population))]
    return population

def Determine_Dom_CnM(population):
    for i in range(0, len(population)):
        for j in range(0, len(population)):
            if i != j:
                if population[i].fit > population[j].fit:    
                    population[j].bool = 'T'
                    break
                elif population[i].fit == population[j].fit:
                    if population[i].age < population[j].age:
                        population[i].bool = 'T'
                    elif population[i].age == population[j].age:
                        if 0.5 > np.random.random():
                            population[i].bool = 'T'
            else:
                continue
    
    
    return population

def Determine2_Dom_CnM(population):
    for individual1 in population:
        for individual2 in population:
            if individual1 != individual2:
                if individual2.fit <= individual1.fit: 
                    if individual2.age >= individual1.age:
                        if individual1.fit == individual2.fit and individual1.age == individual2.age:
                            if individual2.uid < individual1.uid:    
                            #print "fit i/j =", population[i].fit, population[j].fit
                                individual2.bool = 'T'
                        else: 
                            individual2.bool = 'T'
                else:
                    continue
    return population
    
#def set_id(population):
#    for i in range(1, len(population)):
#        population[i].id = population[i-1].id + 1 
#    return population
    
def dominate_CnM(population):
    print "fit", [population[i].fit for i in range(0, len(population))][:10] 
    population = Determine2_Dom_CnM(population)
    print "dom", [population[i].bool for i in range(0, len(population))]
    
    population = sort_by_dom_CnM(population)
    population = sort_by_fit_CnM(population)
    #print len(population)
    population = del_doms_CnM(population)
    #print len(population)
    #population = AGE_individuals(population, generation)  
    print "age", [population[i].age for i in range(0, len(population))]
    print "pop-len =", len(population)
    #print [population[i].fit for i in range(0, len(population))]              
    
    return population
    

def initial_CnM(POP_SIZE, num_timesteps, training_list):
    oldpopulation = [Individual() for i in range(0, POP_SIZE)]
    oldpopulation = run_training_CnM(oldpopulation, num_timesteps, training_list)
    newpopulation = dominate_CnM(oldpopulation)
    return newpopulation
    
def secondary_CnM(newpopulation, POP_SIZE, num_timesteps, training_list):
    newpopulation = run_training_second_CnM(newpopulation, POP_SIZE, num_timesteps, training_list)
    newpopulation = dominate_CnM(newpopulation)
    return newpopulation
    
def iterations_CnM(generations, POP_SIZE, num_timesteps):
    Send_Values_ToFile_two([num_timesteps], 'timesteps.txt')
    training_list, test_list = positions()
    for i in range(0, generations):
        print "Gen =", i
        if i == 0:
            newpopulation = initial_CnM(POP_SIZE, num_timesteps, training_list)
            newpopulation = AGE_individuals_CnM(newpopulation)
            newpopulation2 = newpopulation  
        if i > 0:
            newpopulation = secondary_CnM(newpopulation, POP_SIZE, num_timesteps, training_list)
            newpopulation = AGE_individuals_CnM(newpopulation)  
    
    run_testing_CnM(test_list, newpopulation, num_timesteps) 
    #plt.show()
    return newpopulation #, newpopulation2
    

if __name__ == "__main__":
    population_dict = {}
    for i in range(0,1):
        newpop = iterations_CnM(5, POP_SIZE, num_timesteps)
        population_dict[i] = newpop
    #newpop_dict = {}
    #for i in range(0,10):
    #    newpop_dict[i] = iterations_lazy(750, POP_SIZE)

        #age_list = [newpop[i].age for i in range(0, len(newpop[i]))]

        #fit_list[i] = [newpop[i].fit for i in range(0, len(newpop[i]))]

        #bool_list[i] = [newpop[i].bool for i in range(0, len(newpop[i]))]

        #id_list[i] = [newpop[i].uid for i in range(0, len(newpop[i]))]

    parent = newpop[0].ANN
    filename = "weights_Matrix.csv"
    Send_Values_ToFile(parent, filename)
    os.system('./AppRagdollDemo')
    #print newpop[0].test


