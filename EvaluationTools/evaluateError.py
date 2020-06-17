import matplotlib.pyplot as plt
import numpy as np
import glob

listOfFiles = glob.glob("/media/jose/Nuevo vol/FinalVersionResults/SeqOrgAbd/ErrorGTs*")
arrErrors = []

for filei in listOfFiles:
    arrErrors.append(np.loadtxt(filei))
    
meanError = []  
for error in arrErrors:
    meanError.append(np.mean(error))

plot1 = plt.plot(meanError)
plt.grid(True)
plt.ylabel('some numbers')
plt.show()

n, bins, patches = plt.hist(x=arrErrors[200], bins='auto', color='#0504aa',
                            alpha=0.7, rwidth=0.85)
plt.grid(axis='y', alpha=0.75)
plt.xlabel('Value')
plt.ylabel('Frequency')
plt.title('My Very Own Histogram')
plt.text(23, 45, r'$\mu=15, b=3$')
maxfreq = arrErrors[200].max()
# Set a clean upper y-axis limit.
plt.ylim(ymax=np.ceil(maxfreq / 10) * 10 if maxfreq % 10 else maxfreq + 10)
plt.show()
