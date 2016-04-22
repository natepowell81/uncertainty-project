
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math as m
import os.path
import sys
import time
import csv
import pickle
from copy import deepcopy

NUM_INPUTS_ANN = 4
NUM_OUTPUTS_ANN = 26
POP_SIZE = 20     ############################## POP_SIZE
BIG_NUMBER = 999999999
timesteps = [200]
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
                
            #p[i][j] = 0.0
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

def sort_by_dom(population):
    pop_list = []
    for individual in population:
        pop_list.append((individual.bool, individual))
    pop_list.sort()
    pop = []
    for ind in pop_list:
        pop.append(ind[1])
    return pop
    
def sort_by_fit(population):
    fit_list = []
    for individual in population:
        fit_list.append((individual.fit, individual))
    fit_list.sort()
    pop = []
    for ind in fit_list:
        pop.append(ind[1])
    return pop

def AGE_individuals(population):
    for individual in population:
        individual = individual.age_()
    return population
        
def Inject(population):
    population.append(Individual())
    return population        
    
def Fill(population, POP_SIZE):
    new_individuals = []
    for i in range(0, POP_SIZE - len(population) - 1):
        individual = np.random.choice(population, 1, replace=True)
        individual2 = Individual()
        individual2.ANN = individual[0].ANN
        individual2 = individual2.getMutatedVersion()
        new_individuals.append(individual2)
    new_individuals = Inject(new_individuals)
    return new_individuals
    
    
def run_lazy(population, num_timesteps):
    #move_list = MatrixCreate2(1, len(population))
    for individual in population:
        parent = individual.getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        move_list_2 = MatrixCreate2(1, 4)
        for j in range(0, 4):
            runner(j)
            mvect = open('m_vector.txt', 'r')
            mvect_list = []
            for line in mvect:
                mvect_list.append(float(line))
            move = sum(mvect_list)
            #print np.corrcoef(mvect_list, gvect_list)[0,1]            
            individual.fit = move / float(num_timesteps)
            #print population[i].getErrors()
            move_list_2[0][j] = individual.fit
        #print cat_list_2, corr_list_2
        #move_list[0][i] = sum(move_list_2[0][:])/4.0
        individual.fit = sum(move_list_2[0][:])/4.0
        #print "move-fit =",  population[i].fit
        #print "age =", population[i].age
        #print "Dom =", population[i].bool
        
    #print cat_list, corr_list
    #print "pop-len =", len(population)
    return population
    
def run_second_lazy(population, POP_SIZE, num_timesteps):  #NEW POPULATION ISNT THE SIZE 10
    move_list = []
    new_individuals = Fill(population, POP_SIZE)
    print "num new individuals =", len(new_individuals)
    for individual in new_individuals:
        #randomNum = np.random.random()
        #if randomNum > 0.5:
        parent = individual.ANN
        #parent = parent.getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        move_list_2 = MatrixCreate2(1, 4)
        for j in range(0, 4):
            runner(j)
            mvect = open('m_vector.txt', 'r')
            mvect_list = []
            for line in mvect:
                mvect_list.append(float(line))
            move = sum(mvect_list)
            #print np.corrcoef(mvect_list, gvect_list)[0,1] 
            individual.fit = move / float(num_timesteps)
            move_list_2[0][j] = individual.fit
        move_list.append(sum(move_list_2[0][:])/4.0)
        individual.fit = sum(move_list_2[0][:])/4.0
        #print "move-fit =", individual.fit
        #print "age =", individual.age
        #print "Dom =", individual.bool
    
    #print "o", population
    population = population + new_individuals
    #print "p", population
    return population

def del_doms(population):
    #print [population[i].bool for i in range(0, len(population))]
    population = [individual for individual in population if individual.bool != 'T']
    #print [population[i].bool for i in range(0, len(population))]
    return population

def Determine_Dom(population):
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

def Determine2_Dom(population):
    for individual1 in population:
        for individual2 in population:
            if individual1 != individual2:
                if individual2.fit >= individual1.fit: 
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
    
def dominate_lazy(population):
    print "fit", [population[i].fit for i in range(0, len(population))][0:10] 
    population = Determine2_Dom(population)
    print "dom", [population[i].bool for i in range(0, len(population))]
    
    population = sort_by_dom(population)
    population = sort_by_fit(population)
    #print len(population)
    population = del_doms(population)
    #print len(population)
    #population = AGE_individuals(population, generation)  
    print "age", [population[i].age for i in range(0, len(population))]
    print "pop-len =", len(population)
    #print [population[i].fit for i in range(0, len(population))]              
    
    return population
    

def initial_lazy(POP_SIZE, num_timesteps):
    oldpopulation = [Individual() for i in range(0, POP_SIZE)]
    oldpopulation = run_lazy(oldpopulation, num_timesteps)
    newpopulation = dominate_lazy(oldpopulation)
    return newpopulation
    
def secondary_lazy(newpopulation, POP_SIZE, num_timesteps):
    newpopulation = run_second_lazy(newpopulation, POP_SIZE, num_timesteps)
    newpopulation = dominate_lazy(newpopulation)
    return newpopulation
    
def iterations_lazy2(generations, POP_SIZE, num_timesteps):
    pop_dict = {}
    Send_Values_ToFile_two(timesteps, 'timesteps.txt')
    for i in range(0, generations):
        print "Gen =", i
        if i == 0:
            newpopulation = initial_lazy(POP_SIZE, num_timesteps)
            newpopulation = AGE_individuals(newpopulation)
            #newpopulation2 = newpopulation
            pop_dict[i] = newpopulation
        if i > 0:
            newpopulation = secondary_lazy(newpopulation, POP_SIZE, num_timesteps)
            newpopulation = AGE_individuals(newpopulation)
            pop_dict[i] = newpopulation
        
    return newpopulation, pop_dict #, newpopulation2

if __name__ == "__main__":

    newpop, dictionary = iterations_lazy2(150, POP_SIZE, num_timesteps)

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
























