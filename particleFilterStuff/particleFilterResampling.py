'''Takes two arguments: A list of 2-tuples (pose, weight), and the number
of samples that are supposed to be output.'''

import random
import math

def resample(particles, n):
    totalWeight = 0
    for i in range(len(particles)):
        totalWeight = totalWeight + particles[i][1]
    rands = []
    for i in range(n + 1):
        rands.append(random.uniform(0.0, totalWeight))
    sortedRands = sorted(rands)
    counter = 0
    rCounter = 0
    output = []
    lengthOfOutput = 0
    sumSoFar = particles[0][1]
    while (lengthOfOutput < n):
        if(sortedRands[rCounter] < sumSoFar):
            output.append(particles[counter])
            rCounter = rCounter + 1
            lengthOfOutput = lengthOfOutput + 1
        else:
            counter = counter + 1
            sumSoFar = sumSoFar + particles[counter][1]
    return output

'''Below is the alternative method of resampling given in the paper I read.
It has the advantage of being much less random, so the results will be
more consistent.  The downside is that you can't guarantee how big the output
will be - probably about the same size as the input, but not exactly.'''

def liuResample(particles):
    rooted = []
    n = len(particles)
    total = 0
    output = []
    for i in range(n):
        rooted.append(math.sqrt(particles[i][1]))
        total = total + rooted[i]
    factor = n/total
    for i in range(n):
        aj = (rooted[i] * factor)
        if(aj > 1):
            k = math.floor(aj)
            for j in range(k):
                output.append(particles[i])
        else:
            if(random.uniform(0.0, 1.0) < aj):
                output.append(particles[i])
    return output
    

'''Test'''
print(resample([('a', 12), ('b', 2), ('c', 5)], 5))
#print(liuResample([('a', 12), ('b', 2), ('c', 5)]))
