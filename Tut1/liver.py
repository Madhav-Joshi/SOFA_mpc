import Sofa

def createScene(root):
    root.gravity=[0,0,0]
    root.dt=0.01
    root.addObject("DefaultAnimationLoop")
    root.addObject("MeshGmshLoader",name="meshLoaderCoarse",filename="mesh/liver1.msh")

    # New Node
    node1 = root.addChild("Liver") # Name of child node1
    node1.addObject("EulerImplicitSolver") # Solver
    node1.addObject("CGLinearSolver",iterations="200",tolerance="1e-09",threshold="1e-09")

    node1.addObject("TetrahedronSetTopologyContainer",name="topo",src="@../meshLoaderCoarse")
    node1.addObject("TetrahedronSetGeometryAlgorithms",template="Vec3d",name="GeomAlgo")

    node1.addObject("MechanicalObject",template="Vec3d",name="MechanicalModel",showObject="1")
    node1.addObject("TetrahedronFEMForceField",name="FEM",youngModulus="1000",poissonRatio="0.4")

    node1.addObject("MeshMatrixMass",massDensity="1")
    node1.addObject("ConstantForceField",totalForce="1 0 0")

    # A subnode of Liver
    subnode = node1.addChild("Visual")
    subnode.addObject("OglModel",name="VisualModel",src="@../../meshLoaderCoarse")
    subnode.addObject("IdentityMapping",name="Mapping",input="@../MechanicalModel",output="@VisualModel")
