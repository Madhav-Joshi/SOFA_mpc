import numpy as np


def lift(x): # Basically basis psi from the Koopman paper
    '''Takes n dimensional input and lifts it to N dimensional space according to specific basis set'''
    # Do something here for 3rd and 4th degree as well
    n = np.shape(x)[0] # dimension of state x
    X = x
    def sumi(x):
        return int(x*(x+1)/2)
    def sumisq(x):
        return int(x*(x+1)*(2*x+1)/6)
    def sumsumi(x):
        return int((sumisq(x)+sumi(x))/2)
    for i in range(n): # Add second order nonlinearities
        X = np.block([[X],[x[i,:]*x[i:,:]]])
    X0 = X
    for i in range(n): # Add third order non linearities
        X = np.block([[X],[x[i,:]*X0[n+sumi(n)-sumi(n-i):,:]]])
    X1 = X
    for i in range(n): # Add Fourth order non linearities
        X = np.block([[X],[x[i,:]*X1[np.shape(X0)[0]+sumsumi(n)-sumsumi(n-i):,:]]])
    return X


class predictor_construction():
    def __init__(self,X,X_lift,Y_lift,U):
        AB = Y_lift@np.linalg.pinv(np.block([[X_lift],[U]]))
        self.A = AB[:,:X_lift.shape[0]]
        self.B = AB[:,X_lift.shape[0]:]
        self.C = X@np.linalg.pinv(X_lift)


def predict_traj(predictor,U,X0):
    A = predictor.A
    B = predictor.B
    C = predictor.C

    n = U.shape[1]
    X_lift0 = lift(X0)
    pred_lift = np.zeros((X_lift0.shape[0],n+1))
    pred_lift[:,:1] = X_lift0

    for i in range(1,n+1):
        pred_lift[:,i:i+1] = A@pred_lift[:,i-1:i]+B@U[:,i-1:i]

    return C@pred_lift