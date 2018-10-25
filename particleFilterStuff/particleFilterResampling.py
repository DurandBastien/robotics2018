'''Takes two arguments: A list of 2-tuples (pose, weight), and the number
of samples that are supposed to be output.'''

import random

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

'''Test'''
print(resample([('a', 12), ('b', 2), ('c', 5)], 50))
