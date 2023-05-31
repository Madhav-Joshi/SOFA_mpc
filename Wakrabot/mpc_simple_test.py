import numpy as np
from dataclasses import dataclass
from mpc_func import LinearMPC,lift
import matplotlib.pyplot as plt 

# we specify x0
# we get     [u0 u1 ... u(N-1)]
# and we get [x0 x1 ... x(N-1) xN]
@dataclass
class WakrabotProblem:
    T = 0.05 # Horizon duration
    N = 10 # Number of timesteps

    Xref = np.loadtxt('./Wakrabot/data_files/control_files/desired_trajectory.csv', dtype = float, delimiter=',')
    A = np.loadtxt('./Wakrabot/data_files/systemMatrix.csv',delimiter=',') # depends on dt
    B = np.loadtxt('./Wakrabot/data_files/controlMatrix.csv',delimiter=',') # depends on dt

    e = np.array([[1,1,1,0,0,0]], dtype = float).T*5
    D = np.zeros([e.shape[0],B.shape[1]],dtype=float)
    D[:6,:] = np.array([[1,0,0],[0,1,0],[0,0,1],[-1,0,0],[0,-1,0],[0,0,-1]])
    C = np.zeros((e.shape[0],A.shape[0])) # constraints : e.shape[0]
    
    x0 = np.array([[0,0,0,0,0]]).T
    xref = np.append(x0,Xref[:,1:N],axis=1) # Next points to go to from x0 
    
    wxt = 1 #1e4
    wxc = 1 #1e6
    wu = 10**-4.3
    wdu = 10**-1

class WakrabotMPC(LinearMPC): # We can build the MPC problem here from the bare minimum data about the problem and then initialize the super
    def __init__(self, problem: WakrabotProblem): 
        super().__init__(A = problem.A, # Lifted system predicted
                        B = problem.B, # Lifted system predicted
                        C = problem.C, 
                        D = problem.D, 
                        e = problem.e, 
                        x0 = lift(problem.x0), 
                        xref = lift(problem.xref), 
                        T = problem.T, 
                        N = problem.N, 
                        wxt = problem.wxt, 
                        wxc = problem.wxc, 
                        wu = problem.wu,
                        wdu = problem.wdu)


class CableController():
    def __init__(self,i=0,N=10,wxt=1,wxc=1,wu=1,curr_coord=0):
        self.problem = WakrabotProblem()
        self.problem.N = N
        self.i = i
        try:
            if curr_coord==0: self.curr_coord = self.problem.Xref[:,self.i:self.i+1]
        except:
            self.curr_coord = curr_coord

        self.problem.wxt=wxt
        self.problem.wxc=wxc
        self.problem.wu=wu
        # Set initial and reference point
        self.problem.x0 = self.curr_coord
        self.problem.xref = self.problem.Xref[:,self.i:self.i+self.problem.N+1] # np.append(self.problem.x0,self.problem.Xref[:,self.i+1:self.i+self.problem.N+1],axis=1)

    def scale_problem(self, Sx:np.ndarray=np.array([1]), Su:np.ndarray=np.array([1])):
        n = self.problem.x0.shape[0] # self.problem.A.shape[0]
        m = self.problem.B.shape[1]

        ## Handle errors and add functionalities
        if len(Sx.shape)!=1 or len(Su.shape)!=1:
            raise Exception("Please provide 1D numpy array")
        #### If only one weight is provided then it is applied to all states
        if len(Sx)==1:
            Sx = np.ones(n)*Sx[0]
        elif len(Sx)<=n: #### If some weights are provided, they are applied to initially and remaining are 1
            temp_ones = np.ones([n])
            temp_ones[:len(Sx)] = Sx
            Sx = temp_ones
            del temp_ones 
        else:
            raise Exception("More Sx weights assigned than the dimension of the states in the problem")
        
        if len(Su)==1:
            Su = np.ones(m)*Su[0]
        elif len(Su)<=m:
            temp_ones = np.ones([m])
            temp_ones[:len(Su)] = Su
            Su = temp_ones
            del temp_ones 
        else:
            raise Exception("More Su weights assigned than the dimension of the control inputs in the problem")
        
        self.Sx,self.Su = Sx,Su
        self.problem.x0 = np.diag(Sx)@self.problem.x0
        self.problem.xref = np.diag(Sx)@self.problem.xref
        self.problem.A = np.diag(lift(np.array([Sx]).T).flatten())@self.problem.A@np.diag(lift(np.array([Sx]).T).flatten()**-1)
        self.problem.B = np.diag(lift(np.array([Sx]).T).flatten())@self.problem.B@np.diag(Su**-1)

        self.problem.C = self.problem.C@np.diag(lift(np.array([Sx]).T).flatten()**-1)
        self.problem.D = self.problem.D@np.diag(Su**-1)

    def solveMPC(self): 
        ## Solve mpc
        self.mpc = WakrabotMPC(self.problem)
        self.U = self.mpc.U
        self.Xf = self.mpc.states
        
        print(self.U)
        return

    def descale_problem(self):
        try:
            self.U = np.diag(self.Su**-1)@self.U
            self.Xf = np.diag(lift(np.array([self.Sx]).T).flatten()**-1)@self.Xf
        except:
            raise Exception("Run the scale_problem and solveMPC method first")
        # self.U = np.diag(self.Su**-1)@self.U
        # self.Xf = np.diag(lift(np.array([self.Sx]).T).flatten()**-1)@self.Xf

if __name__=="__main__":    
    c = CableController(0,4,1,0.1,0.01)
    c.scale_problem()
    c.solveMPC()
    c.descale_problem()
    print(c.mpc.U)
    # print(c.problem.x0)
    # print(c.Xf)
