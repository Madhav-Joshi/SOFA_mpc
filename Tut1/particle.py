# Most basic python arrangement with some missing components like anumation loop which are added by default
import Sofa

def createScene(node):

    node.gravity=[0,0,0]
    node.name="root"

    childNode = node.addChild("Particle")

    childNode.addObject('EulerImplicitSolver')
    childNode.addObject('CGLinearSolver',iterations=200,tolerance="1e-09")
    childNode.addObject('MechanicalObject',template="Rigid3d",name="myParticle",position="0 0 0 0 0 0 1",showObject="1")
    childNode.addObject('UniformMass',totalMass="1")
    childNode.addObject('ConstantForceField',name="CFF",totalForce="1 0 0 0 0 0")