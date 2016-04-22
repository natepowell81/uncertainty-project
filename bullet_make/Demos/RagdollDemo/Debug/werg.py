from lazy_bot import Individual
from copy import deepcopy

x = Individual()
y = Individual()

#x.ANN = y.ANN

#x.ANN = 0

individual2 = Individual()
individual2.ANN = x.ANN

individual2.getMutatedVersion()

print individual2.ANN == x.ANN

