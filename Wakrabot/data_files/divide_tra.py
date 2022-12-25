import numpy as np

forces = np.loadtxt('./Wakrabot/data_files/traversing_forces.csv',delimiter=',')
forces = np.block([[0,0,0],[forces]])
div = 40 # number of pieces to divide consecutive forces
x = np.zeros(((forces.shape[0]-1)*div,3))
f_diff = forces[1:,:]-forces[:-1,:]

for i in range(div):
    x[i::div,:] = forces[:-1,:]+f_diff*i/div
print(x.shape)
np.savetxt('./Wakrabot/data_files/divide_tra.csv',x,delimiter=',')