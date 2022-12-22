import Sofa
from wakrabot_cables_controller import CableController

# Choose in your script to activate or not the GUI
USE_GUI = True
vol_mesh_path = "D:\Sofa\Tut1\mesh\Wakrabot_final_SI.msh"
# Control varible to be used for cable
Control_variable = 'force' # 'displacement' also works

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
                            "Sofa.Component.AnimationLoop", # Needed to use components [FreeMotionAnimationLoop]
                            "Sofa.Component.Visual", # Needed to use components [VisualStyle]
                            "Sofa.Component.Constraint.Lagrangian.Solver", # Needed to use components [GenericConstraintSolver]
                        ])  
    root.gravity=[0,-9.810,0]
    root.dt=0.05 # sets the units for time
    # root.addObject("DefaultAnimationLoop")
    root.addObject("FreeMotionAnimationLoop")
    root.addObject("DefaultVisualManagerLoop")
    root.addObject('GenericConstraintSolver', tolerance=1e-5, maxIterations=100)
    
    root.addObject('VisualStyle', displayFlags="showInteractionForceFields showVisual")
    root.addObject("MeshGmshLoader",name="meshLoaderCoarse",filename=vol_mesh_path)
    
    # New Node
    wakra = root.addChild("Wakrabot") # Name of child wakra
    wakra.addObject("EulerImplicitSolver", name='odesolver', rayleighMass=0.1, rayleighStiffness=0.1) # Solver
    # wakra.addObject("CGLinearSolver",iterations="200",tolerance="1e-09",threshold="1e-9")
    wakra.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')

    wakra.addObject("TetrahedronSetTopologyContainer",name="topo",src="@../meshLoaderCoarse")
    wakra.addObject("TetrahedronSetGeometryAlgorithms",template="Vec3d",name="GeomAlgo")

    wakra.addObject("MechanicalObject",template="Vec3d",name="MechanicalModel",showObject="1",translation="0 0 0")
    wakra.addObject("TetrahedronFEMForceField",name="FEM",youngModulus="1000000000",poissonRatio="0.1")

    # wakra.addObject("MeshMatrixMass",massDensity="1000")
    wakra.addObject("UniformMass",totalMass="1") # Approximation

    wakra.addObject('BoxROI', name='roi', box=[-0.050, -0.0023, -0.050, 0.050, 0.0001, 0.050], drawBoxes=True)
    # RestShapeSpringsForceField is one way in Sofa to implement fixed point constraint.
    # Here the constraints are applied to the DoFs selected by the previously defined BoxROI
    wakra.addObject('RestShapeSpringsForceField', points=wakra.roi.indices.getLinkPath(), stiffness=1e12)
    
    wakra.addObject('GenericConstraintCorrection')

    ##########################################
    #######       Cable       ################
    ##########################################
    ############  Cable0    ##################
    cable0 = wakra.addChild('cable0')

    number_of_cable_through_points = 14
    valuetype = Control_variable
    cable0.addObject('MechanicalObject',
                    position=[[0,-0.022*j-0.002,0.04599] for j in range(number_of_cable_through_points)],
                    rotation='0 0 0')

    cable0.addObject('CableConstraint', name="CableActuator0",
                    indices=list(range(0, number_of_cable_through_points)),
                    minForce=0,  # Set that the cable0 can't push
                    pullPoint=[0, 0.022  , 0.04599],
                    value=0,#initialValue
                    valueType=valuetype, # 'displacement' or 'force'
                    hasPullPoint=True)

    cable0.addObject('BarycentricMapping',name="mech_baryCentricMap")
    
    ############  Cable1    ##################
    cable1 = wakra.addChild('cable1')

    number_of_cable_through_points = 14
    cable1.addObject('MechanicalObject',
                    position=[[0,-0.022*j-0.002,0.04599] for j in range(number_of_cable_through_points)],
                    rotation='0 120 0')

    cable1.addObject('CableConstraint', name="CableActuator1",
                    indices=list(range(0, number_of_cable_through_points)),
                    minForce=0,  # Set that the cable0 can't push
                    pullPoint=[0.03982, 0.022  , -0.02299],
                    value=0, #initialValue
                    valueType=valuetype, # or 'force'
                    hasPullPoint=True)

    cable1.addObject('BarycentricMapping',name="mech_baryCentricMap")
    
    ############  Cable2    ##################
    cable2 = wakra.addChild('cable2')

    number_of_cable_through_points = 14
    cable2.addObject('MechanicalObject',
                    position=[[0,-0.022*j-0.002,0.04599] for j in range(number_of_cable_through_points)],
                    rotation='0 240 0')

    cable2.addObject('CableConstraint', name="CableActuator2",
                    indices=list(range(0, number_of_cable_through_points)),
                    minForce=0,  # Set that the cable0 can't push
                    pullPoint=[-0.03982, 0.022  , -0.02299],
                    value=0,#initialValue
                    valueType=valuetype, # or 'force'
                    hasPullPoint=True)

    cable2.addObject('BarycentricMapping',name="mech_baryCentricMap")
    
    wakra.addObject(CableController(name="CableController", wakrabot=wakra, rootnode = root))
    
    ############################################
    ######  Visualisation Node #################
    ############################################

    # A subnode visu_wakra of wakra
    visu_wakra = wakra.addChild("Visual_wakra")
    visu_wakra.addObject("OglModel",name="VisualModel_wakra",src="@../../meshLoaderCoarse",color="green")
    visu_wakra.addObject("IdentityMapping",name="Mapping_wakra",input="@../MechanicalModel",output="@VisualModel_wakra")
    
    return root

def main():
    import Sofa.Gui
    root = Sofa.Core.Node("root") # Create the root node
    # Call the below 'createScene' function to create the scene graph
    createScene(root)
    # Once defined, initialization of the scene graph
    Sofa.Simulation.init(root)
    # Run the simulation for 10 time steps
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
        # Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        # Initialization of the scene will be done here
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()
        print("GUI was closed")
    print("Simulation is done.")

if __name__=="__main__":
    main()