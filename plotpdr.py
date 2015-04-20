import matplotlib.pyplot as plt
lambdavals = [3.5, 4.0, 4.5, 5.0]
lambdavals = [1/x for x in lambdavals]
pdr = [0.008139093,0.00932193,0.0096013015,0.010214717]
plt.plot(lambdavals, pdr, marker='o')
plt.xlabel('Average no. of Vehicles')
plt.ylabel('PDR')
plt.savefig('vehicle.png', bbox_inches='tight')
plt.clf()
