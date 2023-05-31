#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa.Core
import numpy as np
from dataclasses import dataclass
from mpc_func import LinearMPC,lift

@dataclass
class WakrabotProblem:
    T = 0.05 # Horizon duration
    N = 10 # Number of timesteps

    A = np.loadtxt('./Wakrabot/data_files/systemMatrix.csv',delimiter=',')
    B = np.loadtxt('./Wakrabot/data_files/controlMatrix.csv',delimiter=',')
  

    duty_Ulimit, duty_Llimit = 0.62,0.09
    len_upper_limit, len_lower_limit = 0.385, 0.225

    e = np.array([[duty_Ulimit,duty_Ulimit,duty_Ulimit,
                -duty_Llimit,-duty_Llimit,-duty_Llimit,
                 len_upper_limit,len_upper_limit,len_upper_limit,
                -len_lower_limit,-len_lower_limit,-len_lower_limit]], dtype = float).T

    D = np.zeros([e.shape[0],B.shape[1]],dtype=float)
    D[0:3,0:3]=np.eye(3)*1.0
    D[3:6,0:3]=-np.eye(3)*1.0
    
    C = np.zeros((e.shape[0],A.shape[0])) # #constraints : e.shape[0]
    
    C[6:9,2:5]=np.eye(3)
    C[9:12,2:5]=-np.eye(3)


    Xref = np.loadtxt('./Wakrabot/data_files/control_files/desired_trajectory.csv', dtype = float, delimiter=',')[:,:]
    Xref = lift(Xref)
    # Xref = np.loadtxt('./Data/Trajectory_data_for_control.csv',dtype=float,delimiter=',')[:,::10]
    # Xref = lift((Xref - np.array([[220,148]]).T)/220)
    x0 = lift(np.zeros((2,1)))
    xref = Xref[:,0:1] # Next point to go to
    
    wxt = 1 #1e4
    wxc = 0.1 #1e6
    wu = 1e-3

class WakrabotMPC(LinearMPC): # We can build the MPC problem here from the bare minimum data about the problem and then initialize the super
    def __init__(self, problem: WakrabotProblem): 
        super().__init__(A=problem.A, 
                        B = problem.B, 
                        C = problem.C, 
                        D = problem.D, 
                        e = problem.e, 
                        x0 = problem.x0, 
                        xref = problem.xref, 
                        T = problem.T, 
                        N = problem.N, 
                        wxt = problem.wxt, 
                        wxc = problem.wxc, 
                        wu = problem.wu)

def reference_trajectory(i,f,n):
    x = np.block([i,f])
    diff = (x[:,1:]-x[:,:-1])/n
    x_ = np.zeros((x.shape[0],(x.shape[1]-1)*n))
    
    for i in range(n):
      x_[:,i::n] = x[:,:-1]+diff*i
    
    return x_

class CableController(Sofa.Core.Controller):
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print('Controller Initialized')
        self.root = kwargs.get("rootnode")
        self.wakra = kwargs.get('wakrabot')
        self.compressed = 0.1
        self.inputValue = [ self.wakra.cable0.CableActuator0.value,
                            self.wakra.cable1.CableActuator1.value,
                            self.wakra.cable2.CableActuator2.value]

        self.problem = WakrabotProblem()

        self.iterNo = 0
        self.count = 0
        self.start_count = 0
        self.sol_inputs = []
        self.sol_curr_coord = []
        self.centre = np.array([[0,0]]).T
        self.prev_inp = np.zeros((3,1)) # No use for now
        self.problem.N = 4
        self.tol = 0.142

        return

    def onKeypressedEvent(self,e):
        inputValue = [  self.wakra.cable0.CableActuator0.value,
                        self.wakra.cable1.CableActuator1.value,
                        self.wakra.cable2.CableActuator2.value]
        
        if self.wakra.cable0.CableActuator0.valueType.value == 'force': scale = 2e-2
        else: scale = 1e-3 # 1e-3 for displacement control

        ctrl0 = inputValue[0].value
        if e["key"] == '=':
            ctrl0 = inputValue[0].value[0] + scale
            
        elif e["key"] == '-':
            ctrl0 = inputValue[0].value[0] - scale
        inputValue[0].value = [ctrl0]

        ctrl1 = inputValue[1].value
        if e["key"] == ']':
            ctrl1 = inputValue[1].value[0] + scale
            
        elif e["key"] == '[':
            ctrl1 = inputValue[1].value[0] - scale
        inputValue[1].value = [ctrl1]

        ctrl2 = inputValue[2].value
        if e["key"] == "'":
            ctrl2 = inputValue[2].value[0] + scale
            
        elif e["key"] == ";":
            ctrl2 = inputValue[2].value[0] - scale
        inputValue[2].value = [ctrl2]
        
        pos = self.wakra.cable0.cable0MO.position.value
        # print('Length =',np.sum(np.linalg.norm(pos[1:]-pos[:-1],axis=1)))
        # print("Cable length value is", [ctrl0,ctrl1,ctrl2])

    def onAnimateBeginEvent(self, event):
        # Read the data and feed it to controller, then apply control inputs returned by the controller
        print(dir(self.wakra.cable0.cable0MO))
        
        
    # called on each animation step
    def onAnimateEndEvent(self, event): 
        
        if self.start_count>50:

            # Store the value of the laser coordinate in an array to see what trajectory it followed
            # print('Laser Coordinates',self.currentLaserCoordinates())
            i = self.iterNo % self.problem.Xref.shape[1] # This was for making the bot move continuously in circles
            # Change pre coord dimensions
            pre_coord = self.currentLaserCoordinates()[::2].reshape((2,1)) - self.centre
            print('pre_coord',pre_coord) ######################################################

            curr_len = self.currentStringLengths().reshape((3,1))
            print('curr_len',curr_len) ######################################################

            pre_coord = np.concatenate((pre_coord,curr_len),axis=0) # Previous state
            
            self.problem.x0 = lift(pre_coord)
            if np.linalg.norm(pre_coord-self.problem.Xref[:5,i:i+1])>self.tol:
                print("ERROR difference",np.linalg.norm(pre_coord-self.problem.Xref[:5,i:i+1]))
                # print('curr_coord, ref',pre_coord,self.problem.Xref[:5,i:i+1])
                self.count += 1

                ## generate reference trajectory
                x_des = reference_trajectory(pre_coord,self.problem.Xref[:5,i:i+1],self.problem.N+1)
                print('x_des',x_des)
                self.problem.xref = lift(x_des).T.reshape(((self.problem.N+1)*self.problem.A.shape[0],1))

                ## Solve mpc
                mpc = WakrabotMPC(self.problem)

                ## Debug
                print('mpc output', mpc.U[:,:1])

                ## Limit change in control inputs range
                # threshold = 300
                delta = (mpc.U[:,:1]-self.prev_inp[:,:1])
                # print('Change in inputs from mpc are',delta.T)
                #if np.max(np.abs(delta))<100: break
                #if any(np.abs(delta)>threshold): mpc.U[:,:1] = prev_inp[:,:1]+delta*threshold/np.max(np.abs(delta)) 
                self.prev_inp = mpc.U[:,:1]
                self.sol_inputs.append(mpc.U[:,:1])

                ## Move the robot
                for i in range(3):
                    # print(f'Force value {i} =',mpc.U[i,0] + self.compressed)
                    self.inputValue[i].value = [mpc.U[i,0] + self.compressed]

            else:
                print(f"While loop completed for iteation {self.iterNo}")
                self.count = 0
                self.iterNo += 1

        else:
            for i in range(3):
                # print(f'Force value {i} =', self.compressed+0.2)
                self.inputValue[i].value = [self.compressed+0.2]
                self.start_count+=1
                # print(self.currentStringLengths())

        return

    def currentLaserCoordinates(self):
        #access the 'position' state vector
        myMOpositions = self.wakra.MechanicalModel.position.value[[1912,1916,1914,1915,1911,1913]]
        
        pts = np.array([    (myMOpositions[0]+myMOpositions[1])/2,
                            (myMOpositions[2]+myMOpositions[3])/2,
                            (myMOpositions[4]+myMOpositions[5])/2,
                            ])
        center = np.mean(pts,axis=0)
        direction = np.cross(pts[2]-pts[0],pts[1]-pts[0])
        direction = direction/np.linalg.norm(direction)
        
        h = 0.400 # height of floor from the base of the wakrabot
        laser_coord = center + (h-(-center[1]))/(-direction[1])*direction
        return laser_coord

    def currentStringLengths(self):
        
        pos0 = self.wakra.cable0.cable0MO.position.value
        pos1 = self.wakra.cable1.cable1MO.position.value
        pos2 = self.wakra.cable2.cable2MO.position.value
        
        len = np.array([np.sum(np.linalg.norm(pos0[1:]-pos0[:-1],axis=1)),
                        np.sum(np.linalg.norm(pos1[1:]-pos1[:-1],axis=1)),
                        np.sum(np.linalg.norm(pos2[1:]-pos2[:-1],axis=1))
                        ])
        return len