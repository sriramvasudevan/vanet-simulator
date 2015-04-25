# Given two logfiles - one with RPR results and the other with the Uniform results - plot Vageesh's graphs of delay, energy, opex and num_rsus vs min_pdr. finalStats is true, printEpisode is true, printTimestep is false, as are other print booleans.
import re
import matplotlib.pyplot as plt
import math

def getvals(filename):
    with open(filename) as f:
        a = f.readlines()

    a = [re.findall(r"[\w.']+", x) for x in a]
    pdr = []
    delay = []
    energy = []
    opex = []
    rsu = []

    for i in range(3,21,5):
        pdr.append(float(a[i][a[i].index('PDR')+1]))
        delay.append(float(a[i][a[i].index('delay')+1]))
        opex.append(float(a[i][a[i].index('OPEX')+1]))
        energy.append(float(a[i][a[i].index('Energy')+1]))
        rsu.append(float(a[i+1][a[i+1].index('RSUs')+1]))

    pdr = [0.01,0.02,0.03,0.04]
    opex = [math.log(x) for x in opex]
    energy = [math.log(x) for x in energy]
    return pdr, delay, energy, opex, rsu

pdrvals = []
delayvals = []
energyvals = []
opexvals = []
rsuvals = []
for i in ['logfile_proposed', 'logfile_uniform']:
    temp = getvals(i)
    pdrvals.append(temp[0])
    delayvals.append(temp[1])
    energyvals.append(temp[2])
    opexvals.append(temp[3])
    rsuvals.append(temp[4])

plt.plot(pdrvals[0],opexvals[0],marker='o',label='Proposed')
plt.plot(pdrvals[1],opexvals[1],marker='x',linestyle='--', color='r',label='Uniform')
plt.xlabel('PDR')
plt.ylabel('OPEX')
plt.legend(loc=4)
plt.savefig('opex.png', bbox_inches='tight')
plt.clf()

plt.plot(pdrvals[0],energyvals[0],marker='o',label='Proposed')
plt.plot(pdrvals[1],energyvals[1],marker='x',linestyle='--', color='r',label='Uniform')
plt.xlabel('PDR')
plt.ylabel('Energy')
plt.legend(loc=4)
plt.savefig('energy.png', bbox_inches='tight')
plt.clf()

plt.plot(pdrvals[0],delayvals[0],marker='o',label='Proposed')
plt.plot(pdrvals[1],delayvals[1],marker='x',linestyle='--', color='r',label='Uniform')
plt.xlabel('PDR')
plt.ylabel('Delay')
plt.legend(loc=1)
plt.savefig('delay.png', bbox_inches='tight')
plt.clf()

plt.plot(pdrvals[0],rsuvals[0],marker='o',label='Proposed')
plt.plot(pdrvals[1],rsuvals[1],marker='x',linestyle='--', color='r',label='Uniform')
plt.ylim([0,12])
plt.xlabel('PDR')
plt.ylabel('RSUs')
plt.legend(loc=4)
plt.savefig('rsu.png', bbox_inches='tight')
plt.clf()

