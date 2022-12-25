import numpy as np

np.random.seed(1)
num_pts = 200
force = np.random.random((num_pts,3))*0.5
np.savetxt('./Wakrabot/data_files/traversing_forces.csv',force,delimiter=',')
print(force)