import Sofa
from CableController import CableController

# Choose in your script to activate or not the GUI
USE_GUI = True

def createScene(root):
    root.addObject("RequiredPlugin", name='SoftRobots')
    root.addObject("RequiredPlugin", name='SofaPython3')
    root.addObject('RequiredPlugin', pluginName=[
                            "Sofa.Component.LinearSolver.Iterative",  # Needed to use components [CGLinearSolver]
                            "Sofa.Component.Mapping.Linear",  # Needed to use components [IdentityMapping]
                            "Sofa.Component.StateContainer",  # Needed to use components [MechanicalObject]
                            "Sofa.Component.Topology.Container.Dynamic",  # Needed to use components [TetrahedronSetGeometryAlgorithms, TetrahedronSetTopologyContainer]
                            "Sofa.Component.IO.Mesh", # Needed to use components [MeshGmshLoader] 
                            "Sofa.Component.Mass", # Needed to use components [MeshMatrixMass] 
                            "Sofa.Component.MechanicalLoad", # Needed to use components [ConstantForceField] 
                            "Sofa.Component.ODESolver.Backward", # Needed to use components [EulerImplicitSolver] 
                            "Sofa.Component.SolidMechanics.FEM.Elastic", # Needed to use components [TetrahedronFEMForceField] 
                            "Sofa.GL.Component.Rendering3D", # Needed to use components [OglModel] 
                            "Sofa.Component.Engine.Select", # Needed to use components [BoxROI]
                            "Sofa.Component.SolidMechanics.Spring", # Needed to use components [RestShapeSpringsForceField]
                            "Sofa.Component.LinearSolver.Direct", # Needed to use components [SparseLDLSolver]
                            "Sofa.Component.Constraint.Lagrangian.Correction", # Needed to use components [GenericConstraintCorrection]
                            "Sofa.Component.AnimationLoop", # Needed to use components [FreeMotionAnimationLoop]
                            "Sofa.Component.Constraint.Lagrangian.Solver", # Needed to use components [GenericConstraintSolver]
                        ])
    root.gravity=[0,-9.81,0]
    root.dt=0.01

    root.addObject("FreeMotionAnimationLoop")
    root.addObject('DefaultVisualManagerLoop')

    root.addObject('GenericConstraintSolver', tolerance=1e-5, maxIterations=100)
    root.addObject("MeshGmshLoader",name="meshLoaderCoarse",filename="./mesh/extrude.msh")

    # New Node
    cyl = root.addChild("Cylinder") # Name of child node
    cyl.addObject("EulerImplicitSolver", name='odesolver', rayleighMass=0.1, rayleighStiffness=0.1) # Solver
    # cyl.addObject("CGLinearSolver",iterations="200",tolerance="1e-09",threshold="1e-09")
    cyl.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')

    cyl.addObject("TetrahedronSetTopologyContainer",name="topo",src="@../meshLoaderCoarse")
    cyl.addObject("TetrahedronSetGeometryAlgorithms",template="Vec3d",name="GeomAlgo")

    cyl.addObject("MechanicalObject",template="Vec3d",name="MechanicalModel",showObject="1",translation="0 -10 0")
    cyl.addObject("TetrahedronFEMForceField",name="FEM",youngModulus="100000",poissonRatio="0.4")

    cyl.addObject("MeshMatrixMass",massDensity="1")
    # cyl.addObject("UniformMass",totalMass="0.1")
    # cyl.addObject("ConstantForceField",totalForce="1 0 0")

    # To facilitate the selection of DoFs, SOFA has a concept called ROI (Region of Interest).
    #  The idea is that ROI component "select" all DoFS that are enclosed by their "region".
    # We use ROI here to select a group of finger's DoFs that will be constrained to stay
    # at a fixed position.
    # You can either use 'BoxROI'...
    cyl.addObject('BoxROI', name='roi', box=[-1.1, -0.3, -1.1, 1.1, 0.1, 1.1], drawBoxes=True)
    # Or 'SphereROI'...
    # cyl.addObject('SphereROI', name='roi', centers=[0, 0, 0], radii=5)
    
    # RestShapeSpringsForceField is one way in Sofa to implement fixed point constraint.
    # Here the constraints are applied to the DoFs selected by the previously defined BoxROI
    cyl.addObject('RestShapeSpringsForceField', points=cyl.roi.indices.getLinkPath(), stiffness=1e12)
    print(cyl.roi.indices)

    cyl.addObject('GenericConstraintCorrection')
    
    ##########################################
    # Cable                                  #
    ##########################################
    
    #  This creates a new node in the scene. This node is appended to the finger's node.
    cable = cyl.addChild('cable')

    #  This creates a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cable it is a set of positions specifying
    #  the points where the cable is passing by.
    cable.addObject('MechanicalObject',
                    position=[
                        [1, -2, 0],
                        [1, -4, 0],
                        [1, -6, 0],
                        [1, -8, 0],
                        [1, -10, 0]])

    # Create a CableConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    # The last index is where the pullPoint is connected.
    cable.addObject('CableConstraint', name="aCableActuator",
                    indices=list(range(0, 5)),
                    # indices=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
                    minForce=0,  # Set that the cable can't push
                    pullPoint=[1.0, 1.0, 0])

    # This creates a BarycentricMapping. A BarycentricMapping is a key element as it will create a bidirectional link
    #  between the cable's DoFs and the finger's one's so that movements of the cable's DoFs will be mapped
    #  to the finger and vice-versa;
    cable.addObject('BarycentricMapping',name="mech_baryCentricMap")

    # This creates a PythonScriptController that permits to programmatically implement new behavior
    #  or interactions using the Python programming language. The controller is referring to a
    #  file named "controller.py".
    cable.addObject(CableController(name="CableController", node=cable))
    
    ############################################
    ######  Visualisation Node #################
    ############################################

    # A subnode visu_cyl of cyl
    visu_cyl = cyl.addChild("Visual_cyl")
    visu_cyl.addObject("OglModel",name="VisualModel_cyl",src="@../../meshLoaderCoarse",color="green")
    visu_cyl.addObject("IdentityMapping",name="Mapping_cyl",input="@../MechanicalModel",output="@VisualModel_cyl")
    
    visu_cyl.addObject('BarycentricMapping',name="visual_baryCentricMap")
    return root

def main():
    import Sofa.Gui 

    #Create the root node
    root = Sofa.Core.Node("root")
    
    # Call the below 'createScene' function to create the scene graph
    createScene(root)
    
    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)

    # Run the simulation for 10 steps
    # for iteration in range(10):
    #     print(f'Iteration #{iteration}')
    #     Sofa.Simulation.animate(root, root.dt.value)

    if not USE_GUI:
        for iteration in range(10):
            print(f'Iteration #{iteration}')
            Sofa.Simulation.animate(root, root.dt.value)
    else:
        # Find out the supported GUIs
        print ("Supported GUIs are: " + Sofa.Gui.GUIManager.ListSupportedGUI(","))
        # Launch the GUI (qt or qglviewer)
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        # Initialization of the scene will be done here
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()
        print("GUI was closed")

    print("Simulation is done.")

    # print("Simulation made 10 time steps. Done")

if __name__=="__main__":
    main()