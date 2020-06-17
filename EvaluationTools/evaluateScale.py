import matplotlib.pyplot as plt
import numpy as np
import glob


scaleVariation = np.loadtxt("/media/jose/Nuevo vol/FinalVersionResults/Mandala1/1/ScaleVariation.txt")

plot1 = plt.plot(scaleVariation[:,1])
plt.grid(True)
plt.ylabel('ScaleVariation')
plt.show()

