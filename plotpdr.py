import matplotlib.pyplot as plt

#lambdavals = [3.5, 4.0, 4.5, 5.0]
#lambdavals = [1/x for x in lambdavals]
cars = [50,100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600]
pdr = [0.004858569276976259,0.004532560631684483,0.004465970224316833,0.004065727496814588,0.003817603121057725,0.0037325556639829388,0.0035411350315464464,0.0033872096217835356, 0.003281951041628785, 0.003060179078176671, 0.0029744985952955506, 0.0027941011997091457]
#pdr = [0.008139093,0.00932193,0.0096013015,0.010214717]
#plt.plot(lambdavals, pdr, marker='o')
plt.plot(cars, pdr, marker='o')
#plt.xlabel('Average no. of Vehicles')
plt.xlabel('No. of Vehicles')
plt.ylabel('PDR')
plt.savefig('vehicle_fixed.png', bbox_inches='tight')
plt.clf()
