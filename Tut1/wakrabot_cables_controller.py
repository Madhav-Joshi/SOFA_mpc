#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa.Core

class CableController(Sofa.Core.Controller):
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print('Controller Initialized')

        self.wakra = kwargs.get('wakrabot')
        # self.cable0 = kwargs.get('cables')[0]#["cable0"]
        # self.cable1 = kwargs.get('cables')[1]
        # self.cable2 = kwargs.get('cables')[2]
        return

    def onKeypressedEvent(self,e):
        inputValue = [  self.wakra.cable0.CableActuator0.value,
                        self.wakra.cable1.CableActuator1.value,
                        self.wakra.cable2.CableActuator2.value]
        # inputValue = [  self.cable0.CableActuator0.value,
        #                 self.cable1.CableActuator1.value,
        #                 self.cable2.CableActuator2.value]

        scale = 1e-2 # 1e-3 for displacement control

        ctrl0 = inputValue[0].value
        if e["key"] == '=':
            ctrl0 = inputValue[0].value[0] + scale
            
        elif e["key"] == '-':
            ctrl0 = inputValue[0].value[0] - scale
            #if ctrl0 < 0:
            #    ctrl0 = 0
        inputValue[0].value = [ctrl0]

        ctrl1 = inputValue[1].value
        if e["key"] == ']':
            ctrl1 = inputValue[1].value[0] + scale
            
        elif e["key"] == '[':
            ctrl1 = inputValue[1].value[0] - scale
            #if ctrl1 < 0:
            #    ctrl1 = 0
        inputValue[1].value = [ctrl1]

        ctrl2 = inputValue[2].value
        if e["key"] == "'":
            ctrl2 = inputValue[2].value[0] + scale
            
        elif e["key"] == ";":
            ctrl2 = inputValue[2].value[0] - scale
            #if ctrl2 < 0:
            #    ctrl2 = 0
        inputValue[2].value = [ctrl2]

        print("Cable length value is", [ctrl0,ctrl1,ctrl2])
        return