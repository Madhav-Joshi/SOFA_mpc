import numpy as np

# Circle
# x^2 + z^2 = 1
# x = r*cos(t), z = y*sin(t)
t = np.linspace(0,np.pi*2,126,endpoint=False)
r = 0.06 # in m 
x_des = np.array([r*np.cos(t),r*np.sin(t)])
des_length = np.ones([3,x_des.shape[1]])*0
x_des = np.concatenate((x_des,des_length),axis=0)
np.savetxt('./Wakrabot/data_files/control_files/desired_trajectory.csv',x_des,delimiter=',')

print(x_des.shape)
'''
import matplotlib.pyplot as plt
plt.plot(x[0],x[1])
plt.show()
'''