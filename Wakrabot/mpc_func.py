import numpy as np
from qpsolvers import solve_qp # Only quadprog solver is used in this file
from typing import Optional
import matplotlib.pyplot as plt

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

class LinearMPC:
    def __init__(self,A,B,C,D,e,x0,xref,T,N: int, wxt, wxc, wu, wdu=0):
        '''
        x(k+1) = A*x(k)+B*u
        '''
        self.A,self.B,self.C,self.D = A,B,C,D
        self.e,self.x0,self.xref,self.T,self.N = e,x0,xref,T,N
        self.wxt,self.wxc,self.wu,self.wdu = wxt,wxc,wu,wdu
        self.build()

    def build(self):
        n,m,o = self.B.shape[0],self.B.shape[1],self.e.shape[0]
        
        if isinstance(self.wxt,np.ndarray):
            temp = np.zeros(n)
            temp[:self.wxt.shape[0]] = self.wxt
            self.wxt = temp
        else:
            self.wxt = np.ones(n,dtype=float)*self.wxt
        if isinstance(self.wxc,np.ndarray):
            temp = np.zeros(n)
            temp[:self.wxc.shape[0]] = self.wxc
            self.wxc = temp
        else:
            self.wxc = np.ones(n,dtype=float)*self.wxc
        if isinstance(self.wu,np.ndarray):
            temp = np.zeros(m)
            temp[:self.wu.shape[0]] = self.wu
            self.wu = temp
        else:
            self.wu = np.ones(m,dtype=float)*self.wu
        self.wu += 2*self.wdu
        self.wu[0] = self.wu[0]-self.wdu
        self.wu[-1] = self.wu[-1]-self.wdu
        Q_tilde, Q_d, R_d = np.diag(self.wxt), np.diag(self.wxc), np.diag(self.wu)
        R_d[:-1,1:] += -self.wdu*np.eye(R_d.shape[0]-1)
        R_d[1:,:-1] += -self.wdu*np.eye(R_d.shape[0]-1) 
        
        Q = np.kron(np.eye(self.N+1),Q_d)
        Q[-n:,-n:] = Q_tilde
        R = np.kron(np.eye(self.N),R_d)

        A_ = np.array(np.block([[np.matrix(self.A)**i] for i in range(self.N+1)]))
        B_ = np.block([
            list(
                np.append(
                    np.array([
                        np.matmul(np.linalg.matrix_power(self.A,j-1-i),self.B) for i in range(j)
                        ]),
                        np.zeros((self.N-j,n,m)),axis=0
                    )
                ) for j in range(1,self.N+1)
            ])
        B_ = np.block([[np.zeros((n,self.N*m))],[B_]])
        # Xref = np.kron(np.ones((self.N+1,1)),self.xref)
        Xref = self.xref.T.reshape(((self.N+1)*self.A.shape[0],1))

        self.P = B_.T@Q@B_+R
        q = B_.T@Q@(A_@self.x0-Xref)
        self.q = q.flatten()

        Cbar = np.block([np.kron(np.eye(self.N),    self.C),np.zeros((self.N*o,n))])
        Dbar = np.kron(np.eye(self.N),self.D)
        ebar = np.kron(np.ones((self.N,1)),self.e)
        self.G = Cbar@B_+Dbar
        h = ebar-Cbar@A_@self.x0
        self.h = h.flatten()

        self.A_ = A_ 
        self.B_ = B_ 

        self.solve()

    def solve(self):
        U = solve_qp(self.P, self.q, self.G, self.h, solver='quadprog',verbose=True)
        try:
            self.U = U.reshape((self.N,int(U.shape[0]/self.N))).T
        except:
            print("MPC did not converge, the output was null")
            self.U = np.zeros((int(self.B_.shape[1]/self.N),self.N))

    @property
    def states(self):
        assert self.U is not None, "you need to solve() the MPC problem first"
        U = self.U.T.reshape((self.U.shape[0]*self.U.shape[1],1))
        X = self.A_@self.x0 + self.B_@U 
        X_ = X.reshape((self.N+1,self.A.shape[0])).T
        return X_

