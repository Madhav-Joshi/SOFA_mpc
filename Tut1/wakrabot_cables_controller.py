#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa.Core
import numpy as np

class CableController(Sofa.Core.Controller):
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print('Controller Initialized')
        self.root = kwargs.get("rootnode")
        self.wakra = kwargs.get('wakrabot')
        return

    def onKeypressedEvent(self,e):
        inputValue = [  self.wakra.cable0.CableActuator0.value,
                        self.wakra.cable1.CableActuator1.value,
                        self.wakra.cable2.CableActuator2.value]
        
        if self.wakra.cable0.CableActuator0.valueType.value == 'force': scale = 1e-2
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

        # print("Cable length value is", [ctrl0,ctrl1,ctrl2])

    def onAnimateBeginEvent(self, event):
        #do whatever you want at the beginning of the step
        t = self.root.time.value
        # print(f'Begin Animate Event Called at {np.round(t,2)}')

    # called on each animation step
    def onAnimateEndEvent(self, event): 
        #access the 'position' state vector
        myMOpositions = self.wakra.MechanicalModel.position.value[[1912,1916,1914,1915,1911,1913]]
        # print the first value of the DOF 0 (Vec3 : x,y,z) x[0] y[0] z[0]
        # print('Coordinates of front',str((myMOpositions[0]+myMOpositions[1])/2))
        # print('Coordinates of right',str((myMOpositions[2]+myMOpositions[3])/2))
        # print('Coordinates of leftt',str((myMOpositions[4]+myMOpositions[5])/2))
        pts = np.array([    (myMOpositions[0]+myMOpositions[1])/2,
                            (myMOpositions[2]+myMOpositions[3])/2,
                            (myMOpositions[4]+myMOpositions[5])/2,
                            ])
        center = np.mean(pts,axis=0)
        direction = np.cross(pts[2]-pts[0],pts[1]-pts[0])
        direction = direction/np.linalg.norm(direction)
        
        h = 0.400 # height of floor from the base of the wakrabot
        laser_coord = center + (h-(-center[1]))/(-direction[1])*direction
        print('Laser Coordinates',laser_coord)
        return