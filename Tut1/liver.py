import Sofa

def createScene(root):
    root.gravity=[0,0,0]
    root.dt=0.01
    root.addObject("DefaultAnimationLoop")
    root.addObject("MeshGmshLoader",name="meshLoaderCoarse",filename="mesh/liver1.msh")

    # New Node
    liver = root.addChild("Liver") # Name of child node
    liver.addObject("EulerImplicitSolver") # Solver
    liver.addObject("CGLinearSolver",iterations="200",tolerance="1e-09",threshold="1e-09")

    liver.addObject("TetrahedronSetTopologyContainer",name="topo",src="@../meshLoaderCoarse")
    liver.addObject("TetrahedronSetGeometryAlgorithms",template="Vec3d",name="GeomAlgo")

    liver.addObject("MechanicalObject",template="Vec3d",name="MechanicalModel",showObject="1")
    liver.addObject("TetrahedronFEMForceField",name="FEM",youngModulus="1000",poissonRatio="0.4")

    liver.addObject("MeshMatrixMass",massDensity="1")
    liver.addObject("ConstantForceField",totalForce="1 0 0")

    # A subnode visu_liver of Liver
    visu_liver = liver.addChild("Visual_liver")
    visu_liver.addObject("OglModel",name="VisualModel_liver",src="@../../meshLoaderCoarse")
    visu_liver.addObject("IdentityMapping",name="Mapping_liver",input="@../MechanicalModel",output="@VisualModel_liver")

    # Heart
    # visu subnode
