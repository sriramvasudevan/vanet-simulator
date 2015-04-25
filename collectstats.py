## A simple script to obtain statitics about PDR obtained when N vehicles are in the system. Set pdr_fixedcars to true in the simulator and compile .java files. To be placed in the same place as all the .class files.
import subprocess

NUM = 20

cars = [i*2 for i in range(25,325,25)]
pdr = [0 for _ in range(25,325,25)]
for _ in range(NUM):
    for num, i in enumerate(range(25, 325, 25)):
        a=subprocess.check_output("java Simulator "+str(i), shell=True)
        pdr[num] += float(a.strip())

pdr = [x/NUM for x in pdr]
print pdr

