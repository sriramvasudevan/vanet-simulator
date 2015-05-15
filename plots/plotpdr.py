# Place results obtained from collectstats.py here to obtain plots. Set pdr equal to the array obtained from collectstats.py, and the car list equal to what was specified in collectstats.py
import matplotlib.pyplot as plt

# Larger values
#cars = [50,100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600]
#pdr = [0.00517391592636689, 0.004642676533650912, 0.004341214356794593, 0.004062467209497342, 0.003834452013372309, 0.0036748974930042614, 0.003500559318841496, 0.0033535311168298243, 0.003208985396682187, 0.0031089735505773785, 0.002991050130959794, 0.0028713495809066176]

# Smaller values
#cars = range(1,100,10)
#pdr = [0.007284932351573753, 0.005689043235139049, 0.005333142503942427, 0.005054424129634901, 0.004843181983516702, 0.0046380066273742585, 0.004527975134101466, 0.00441675307925492, 0.00426814109580231, 0.004190297412267118]

# Combined
#cars = range(1,100,10)
#cars.extend(range(100,650,50))
#pdr = [0.007284932351573753, 0.005689043235139049, 0.005333142503942427, 0.005054424129634901, 0.004843181983516702, 0.0046380066273742585, 0.004527975134101466, 0.00441675307925492, 0.00426814109580231, 0.004190297412267118, 0.004142676533650912, 0.003841214356794593, 0.003562467209497342, 0.003334452013372309, 0.0031748974930042614, 0.003000559318841496, 0.0028535311168298243, 0.002708985396682187, 0.0026089735505773785, 0.002491050130959794, 0.0023713495809066176]
cars = [i*2 for i in range(25,325,25)]
pdr = [0.0048703696185580184, 0.004368343200005859, 0.0041090722079585905, 0.0038269200364247414, 0.003639932198478428, 0.003463462930015736, 0.0032992473061806485, 0.0031559805570611312, 0.0030317732849185594, 0.002906961329354733, 0.0027974451886581255, 0.0027135440929011126]
plt.plot(cars, pdr, marker='o')
plt.xlabel('No. of Vehicles')
plt.ylabel('PDR')
plt.savefig('vehicle_fixed_0001_50_600.png', bbox_inches='tight')
plt.clf()
