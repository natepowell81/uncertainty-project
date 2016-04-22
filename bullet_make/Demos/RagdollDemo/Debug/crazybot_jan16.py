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

NUM_INPUTS_ANN = 4
NUM_OUTPUTS_ANN = 26
POP_SIZE = 10
BIG_NUMBER = 999999999
timesteps = [400]
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

Send_Values_ToFile_two(timesteps, 'timesteps.txt')

def positions():
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

def run_training(population, num_timesteps, training_list):
    for individual in population:
        parent = individual.ANN
        #parent = parent.getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        move_list_2 = MatrixCreate2(1, 7)
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
            #print 'move:', move
            #print np.corrcoef(mvect_list, gvect_list)[0,1]            
            individual.fit = move * move2 #/ float(num_timesteps)
            #print population[i].getErrors()
            move_list_2[0][index] = individual.fit
            #print cat_list_2, corr_list_2
            #move_list[0][i] = sum(move_list_2[0][:])/4.0
            index = index + 1
        individual.fit = sum(move_list_2[0][:])/7.0
    return population

def run_training_second(population, POP_SIZE, num_timesteps, training_list):
    move_list = []
    new_individuals = Fill_spaz(population, POP_SIZE)
    print "num new individuals =", len(new_individuals)
    for individual in new_individuals:
        parent = individual.ANN
        #parent = parent.getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        move_list_2 = MatrixCreate2(1,7)
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
            #print 'move:', move
            #print np.corrcoef(mvect_list, gvect_list)[0,1]            
            individual.fit = move * move2 #/ float(num_timesteps)
            #print population[i].getErrors()
            move_list_2[0][index] = individual.fit
            #print cat_list_2, corr_list_2
            #move_list[0][i] = sum(move_list_2[0][:])/4.0
            index += 1
        individual.fit = sum(move_list_2[0][:])/7.0
    population = population + new_individuals
    return population
            
                    
def run_testing(test_list):
    for individual in population:
        for position in test_list:
            position_size = position           #Center robot should be blind to this
            Send_Values_ToFile_two(position_size, "position_size.csv")
            os.system('./AppRagdollDemo')
            mvect = open('m_vector.txt', 'r')
            mvect_list = []
            for line in mvect:
                mvect_list.append(float(line))
            move = mvect_list[num_timesteps-1] #sum(mvect_list)
            move2 = sum(mvect_list)/float(num_timesteps)
            #print 'move:', move
            #print np.corrcoef(mvect_list, gvect_list)[0,1]            
            individual.fit = move * move2 #/ float(num_timesteps)
            #print population[i].getErrors()
            move_list_2[0][j] = individual.fit
            #print cat_list_2, corr_list_2
            #move_list[0][i] = sum(move_list_2[0][:])/4.0
            individual.fit = sum(move_list_2[0][:])/14.0

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
        self.test = 0.0
        
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

def sort_by_dom_spaz(population):
    pop_list = []
    for individual in population:
        pop_list.append((individual.bool, individual))
    pop_list.sort()
    pop = []
    for ind in pop_list:
        pop.append(ind[1])
    return pop
    
def sort_by_fit_spaz(population):
    fit_list = []
    for individual in population:
        fit_list.append((individual.fit, individual))
    fit_list.sort()
    pop = []
    for ind in fit_list:
        pop.append(ind[1])
    return pop

def AGE_individuals_spaz(population):
    for individual in population:
        individual = individual.age_()
    return population
        
def Inject_spaz(population):
    population.append(Individual())
    return population        
    
def Fill_spaz(population, POP_SIZE):
    new_individuals = []
    for i in range(0, POP_SIZE - len(population) - 1):
        individual = np.random.choice(population, 1, replace=True)
        individual2 = Individual()
        individual2.ANN = individual[0].ANN
        individual2 = individual2.getMutatedVersion()
        new_individuals.append(individual2)
    new_individuals = Inject_spaz(new_individuals)
    return new_individuals
    
    
def run_spaz(population, num_timesteps):
    #move_list = MatrixCreate2(1, len(population))
    for individual in population:
        parent = individual.getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        move_list_2 = MatrixCreate2(1, 14)
        for j in range(0, 14):
            runner(j)
            mvect = open('m_vector.txt', 'r')
            mvect_list = []
            for line in mvect:
                mvect_list.append(float(line))
            move = mvect_list[num_timesteps-1] #sum(mvect_list)
            move2 = sum(mvect_list)/float(num_timesteps)
            #print 'move:', move
            #print np.corrcoef(mvect_list, gvect_list)[0,1]            
            individual.fit = move #/ float(num_timesteps)
            #print population[i].getErrors()
            move_list_2[0][j] = individual.fit
        #print cat_list_2, corr_list_2
        #move_list[0][i] = sum(move_list_2[0][:])/4.0
        individual.fit = sum(move_list_2[0][:])/14.0
        #print "move-fit =",  population[i].fit
        #print "age =", population[i].age
        #print "Dom =", population[i].bool
        
    #print cat_list, corr_list
    #print "pop-len =", len(population)
    return population
    
def run_second_spaz(population, POP_SIZE, num_timesteps):  #NEW POPULATION ISNT THE SIZE 10
    move_list = []
    new_individuals = Fill_spaz(population, POP_SIZE)
    print "num new individuals =", len(new_individuals)
    for individual in new_individuals:
        #randomNum = np.random.random()
        #if randomNum > 0.5:
        parent = individual.ANN
        #parent = parent.getValues()
        filename = "weights_Matrix.csv"
        Send_Values_ToFile(parent, filename)
        move_list_2 = MatrixCreate2(1, 14)
        for j in range(0, 14):
            runner(j)
            mvect = open('m_vector.txt', 'r')
            mvect_list = []
            for line in mvect:
                mvect_list.append(float(line))
            move = mvect_list[num_timesteps-1] #sum(mvect_list)
            move2 = sum(mvect_list)/float(num_timesteps)
            #print 'move:', move
            #print np.corrcoef(mvect_list, gvect_list)[0,1] 
            individual.fit = move #/ float(num_timesteps)
            move_list_2[0][j] = individual.fit
        move_list.append(sum(move_list_2[0][:])/14.0)
        individual.fit = sum(move_list_2[0][:])/14.0
        #print "move-fit =", individual.fit
        #print "age =", individual.age
        #print "Dom =", individual.bool
    
    #print "o", population
    population = population + new_individuals
    #print "p", population
    return population

def del_doms_spaz(population):
    #print [population[i].bool for i in range(0, len(population))]
    population = [individual for individual in population if individual.bool != 'T']
    #print [population[i].bool for i in range(0, len(population))]
    return population

def Determine_Dom_spaz(population):
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

def Determine2_Dom_spaz(population):
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
    
def dominate_spaz(population):
    print "fit", [population[i].fit for i in range(0, len(population))][:10] 
    population = Determine2_Dom_spaz(population)
    print "dom", [population[i].bool for i in range(0, len(population))]
    
    population = sort_by_dom_spaz(population)
    population = sort_by_fit_spaz(population)
    #print len(population)
    population = del_doms_spaz(population)
    #print len(population)
    #population = AGE_individuals(population, generation)  
    print "age", [population[i].age for i in range(0, len(population))]
    print "pop-len =", len(population)
    #print [population[i].fit for i in range(0, len(population))]              
    
    return population
    

def initial_spaz(POP_SIZE, num_timesteps, training_list):
    oldpopulation = [Individual() for i in range(0, POP_SIZE)]
    oldpopulation = run_training(oldpopulation, num_timesteps, training_list)
    newpopulation = dominate_spaz(oldpopulation)
    return newpopulation
    
def secondary_spaz(newpopulation, POP_SIZE, num_timesteps, training_list):
    newpopulation = run_training_second(newpopulation, POP_SIZE, num_timesteps, training_list)
    newpopulation = dominate_spaz(newpopulation)
    return newpopulation
    
def iterations_spaz(generations, POP_SIZE, num_timesteps):
    training_list, test_list = positions()
    for i in range(0, generations):
        print "Gen =", i
        if i == 0:
            newpopulation = initial_spaz(POP_SIZE, num_timesteps, training_list)
            newpopulation = AGE_individuals_spaz(newpopulation)
            newpopulation2 = newpopulation  
        if i > 0:
            newpopulation = secondary_spaz(newpopulation, POP_SIZE, num_timesteps, training_list)
            newpopulation = AGE_individuals_spaz(newpopulation)  
        
    return newpopulation #, newpopulation2
    

if __name__ == "__main__":

    newpop = iterations_spaz(50, POP_SIZE, num_timesteps)

    #newpop_dict = {}
    #for i in range(0,10):
    #    newpop_dict[i] = iterations_lazy(750, POP_SIZE)

        #age_list = [newpop[i].age for i in range(0, len(newpop[i]))]

        #fit_list[i] = [newpop[i].fit for i in range(0, len(newpop[i]))]

        #bool_list[i] = [newpop[i].bool for i in range(0, len(newpop[i]))]

        #id_list[i] = [newpop[i].uid for i in range(0, len(newpop[i]))]

    parent = newpop[2].ANN
    filename = "weights_Matrix.csv"
    Send_Values_ToFile(parent, filename)
    os.system('./AppRagdollDemo')
    


