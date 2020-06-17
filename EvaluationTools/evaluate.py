import matplotlib.pyplot as plt
import numpy as np
import glob

allSequences = []
for i in range(1,6):
    print(i)
    listOfFiles = glob.glob("/media/jose/Nuevo vol/FinalVersionResults/Mandala3/"+str(i)+"/ErrorGTs*")
    arrErrors = []

    for filei in listOfFiles:
        arrErrors.append(np.loadtxt(filei))
    
    for idx, error in enumerate(arrErrors):
        arrErrors[idx] = np.sort(error)
        arrErrors[idx] = arrErrors[idx][:int(error.size*0.80)]

    meanError = []  
    for error in arrErrors:
        meanError.append(np.sqrt(np.mean((error)**2)))

    allSequences.append(meanError)

mergedPlot = []
for idx, error in enumerate(allSequences[0]):
   medianError= []
   for errorSequence in allSequences:
       medianError.append(errorSequence[idx])
   mergedPlot.append(np.median(medianError))

#for plo in allSequences:
 #  plt.plot(plo)

plt.plot(mergedPlot)
plt.grid(True)
plt.ylabel('some numbers')
plt.show()

