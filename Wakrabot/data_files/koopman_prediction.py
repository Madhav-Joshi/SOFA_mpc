import numpy as np
from koopman_func import lift, predictor_construction, predict_traj
import matplotlib.pyplot as plt
import pickle

data = np.loadtxt('./Wakrabot/data_files/simulation_data.csv',delimiter=',')

time = data[:,0]
forces = data[:,1:4].T 
forces = np.block([[forces[:,:-1]],[forces[:,1:]-forces[:,:-1]]])
output = data[:,4:].T # L1,L2,L3,X,Z (all < 1, no need to normalize)

TRAIN_PERCENTAGE = 80
train_index = int(output.shape[1]*TRAIN_PERCENTAGE/100.)
data_train = output[:,:train_index]

X,Y = data_train[:,:-1],data_train[:,1:]
U = forces[:,:X.shape[1]]

predictor = predictor_construction(X,lift(X),lift(Y),U)

# Save predictor object
with open(f'./Wakrabot/data_files/predictor.pickle', 'wb') as file:
    pickle.dump(predictor, file)

# To load the file
# with open(f'./Wakrabot/data_files/predictor.pickle', 'rb') as file2:
#     predictor = pickle.load(file2) 

## Test prediction
offset = 1500
prediction_length = 100

test_start = train_index+offset
X_test = output[:,test_start:test_start+prediction_length]
U_test = forces[:,test_start:test_start+X_test.shape[1]-1]

print(U_test.shape,X_test[:,:1].shape)

predicted = predict_traj(predictor=predictor,U=U_test,X0=X_test[:,:1])
# print(predicted.shape,np.max(np.abs(predicted)))
fig,axs = plt.subplots(1,1)
axs.plot(X_test[0,:],X_test[1,:],label='Original')
axs.scatter(predicted[0,:],predicted[1,:],label='Prediction')
axs.legend()
axs.set(title='X-Y plot of laser trajectory and its Prediction', xlabel='X (m)', ylabel='Z (m)')
plt.show()