## A simple script to obtain statitics about PDR obtained when N vehicles are in the system. Set pdr_fixedcars to true in the simulator and compile .java files. To be placed in the same place as all the .class files.
import subprocess,sys

NUM = 2
vehrange = range(1,25,1)
SINR = sys.argv[1]

pdr = [0 for _ in vehrange]
for _ in range(NUM):
    for num, i in enumerate(vehrange):
        a=subprocess.check_output("java Simulator "+str(i)+" "+SINR, shell=True)
        pdr[num] += float(a.strip())

pdr = [x/NUM for x in pdr]
print pdr

