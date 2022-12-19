import Sofa
import SofaRuntime
import numpy as np

# Choose in your script to activate or not the GUI
USE_GUI = True

class printPosition(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, *kwargs)
        print('Controller Initialized')
        self.tetrameshMO = kwargs.get("MechanicalObject")
        self.root = kwargs.get("rootnode")
        # print(str(self.tetrameshMO.position.value[2][0]))
        # As usual with python one possible way to discover an API is to use python introspection capbalities
        # Eg: list the object's method with dir()
        # print(str(dir(self.tetrameshMO)))
        # print(str(dir(self.root)))
        # See nodes 500, 503 and 507 positions

    def onAnimateBeginEvent(self, event):
        #do whatever you want at the beginning of the step
        t = self.root.time.value
        print(f'Begin Animate Event Called at {np.round(t,2)}')

    #called on each animation step
    def onAnimateEndEvent(self, event):
        #access the 'position' state vector
        myMOpositions = np.round(self.tetrameshMO.position.value,4)
        # print the first value of the DOF 0 (Vec3 : x,y,z) x[0] y[0] z[0]
        print(str(myMOpositions[500]))#[0])+' '+str(myMOpositions[500][1])+' '+str(myMOpositions[500][2]))

def createScene(root):
    root.addObject('RequiredPlugin', pluginName=[
                            "Sofa.Component.Collision.Detection.Algorithm",#/> <!-- Needed to use components [BVHNarrowPhase, BruteForceBroadPhase, DefaultPipeline] --                                                                               ->
                            "Sofa.Component.Collision.Detection.Intersection",#/> <!-- Needed to use components [MinProximityIntersection] -->
                            "Sofa.Component.Collision.Geometry",#/> <!-- Needed to use components [TriangleCollisionModel] -->
                            "Sofa.Component.Collision.Response.Contact",#/> <!-- Needed to use components [DefaultContactManager] -->
                            "Sofa.Component.Constraint.Projective",#/> <!-- Needed to use components [FixedConstraint, FixedPlaneConstraint] -->
                            "Sofa.Component.IO.Mesh",#/> <!-- Needed to use components [MeshGmshLoader] -->
                            "Sofa.Component.LinearSolver.Iterative",#/> <!-- Needed to use components [CGLinearSolver] -->
                            "Sofa.Component.Mapping.Linear",#/> <!-- Needed to use components [IdentityMapping] -->
                            "Sofa.Component.Mass",#/> <!-- Needed to use components [DiagonalMass] -->
                            "Sofa.Component.ODESolver.Backward",#/> <!-- Needed to use components [EulerImplicitSolver] -->
                            "Sofa.Component.SolidMechanics.FEM.Elastic",#/> <!-- Needed to use components [TetrahedralCorotationalFEMForceField] -->
                            "Sofa.Component.SolidMechanics.Spring",#/> <!-- Needed to use components [TriangularBendingSprings] -->
                            "Sofa.Component.StateContainer",#/> <!-- Needed to use components [MechanicalObject] -->
                            "Sofa.Component.Topology.Container.Dynamic",#/> <!-- Needed to use components [TetrahedronSetGeometryAlgorithms, TetrahedronSetTopologyCont                                                                               tainer, TetrahedronSetTopologyModifier, TriangleSetGeometryAlgorithms, TriangleSetTopologyContainer, TriangleSetTopologyModifier] -->
                            "Sofa.Component.Topology.Mapping",#/> <!-- Needed to use components [Tetra2TriangleTopologicalMapping] -->
                            "Sofa.Component.Visual",#/> <!-- Needed to use components [VisualStyle] -->
                            "Sofa.GL.Component.Rendering3D",#/> <!-- Needed to use components [OglModel] -->
                        ])

    root.gravity=[0,-9.81,0]
    root.dt=0.05

    # Enable/ disable options in view tab, see doc/components on sofa website
    root.addObject('VisualStyle', displayFlags="showBehaviorModels showVisual") #  showWireframe
    root.addObject('DefaultAnimationLoop')
    # Basic Components to perform the collision detection
    # This Collision pipeline performs steps related to the collision, mainly collision detection and collision response
    root.addObject('DefaultPipeline', name="DefaultCollisionPipeline", depth="6")
    # Basic broad phase detection of bounding boxes
    root.addObject('BruteForceBroadPhase')
    # Basic Narrow phase of collision detection (Bounding Volume)
    root.addObject('BVHNarrowPhase')
    root.addObject('MinProximityIntersection', name="Proximity", alarmDistance="0.8", contactDistance="0.5")
    root.addObject('DefaultContactManager', name="Response")

    # New node
    tetraMesh = root.addChild("Tetrahedrons Mesh")
    tetraMesh.addObject('MeshGmshLoader', name="loader", filename="mesh/cylinder.msh")
    tetraMesh.addObject('MechanicalObject', src="@loader", template="Vec3d", name="Volume")
    tetraMesh.addObject('EulerImplicitSolver', name="cg_odesolver", printLog="0",  rayleighStiffness="0.1", rayleighMass="0.1")
    tetraMesh.addObject('CGLinearSolver', template="GraphScattered", name="linear solver", iterations="25", tolerance="1e-09", threshold="1e-09")
    tetraMesh.addObject('TetrahedronSetTopologyContainer', src="@loader", name="Container", filename="mesh/cylinder.msh")
    
    # In some simulations, the topology may evolve, see doc/simulation principals/topology
    tetraMesh.addObject('TetrahedronSetTopologyModifier', name="Modifier")
    tetraMesh.addObject('TetrahedronSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo", drawTetrahedra="0", drawColorTetrahedra="1.0 1.0 0.3 1.0")
    
    # Several FEM simulation algorithms, can choose depending on speed and accuracy
    tetraMesh.addObject('TetrahedralCorotationalFEMForceField', template="Vec3d", name="FEM", method="large", poissonRatio="0.3", youngModulus="360", assembling="0")
    
    tetraMesh.addObject('DiagonalMass', template="Vec3d,Vec3d", name="default11", massDensity="0.5")
    tetraMesh.addObject('FixedPlaneConstraint', template="Vec3d", name="default12", direction="0 0 1", dmin="-0.1", dmax="0.1")
    tetraMesh.addObject('FixedConstraint', template="Vec3d", name="default13", indices="0")

    triangleMesh = tetraMesh.addChild("Triangle Mesh")
    triangleMesh.addObject('TriangleSetTopologyContainer', name="Container")
    triangleMesh.addObject('TriangleSetTopologyModifier', name="Modifier")
    triangleMesh.addObject('TriangleSetGeometryAlgorithms', template="Vec3d", name="GeomAlgo")
    triangleMesh.addObject('Tetra2TriangleTopologicalMapping', name="Mapping", input="@../Container", output="@Container")
    triangleMesh.addObject('TriangularBendingSprings', template="Vec3d", name="FEM-Bend", stiffness="300", damping="1")
    triangleMesh.addObject('TriangleCollisionModel', name="Models")

    visu = triangleMesh.addChild("Visu")
    visu.addObject('OglModel', name="Visual", material="Default Diffuse 1 0 0 1 1 Ambient 1 0 0 0.2 1 Specular 0 0 0 1 1 Emissive 0 0 0 1 1 Shininess 0 45")
    visu.addObject('IdentityMapping', template="Vec3d,Vec3d", name="default17", input="@../../Volume", output="@Visual")

    # See nodes 500, 503 and 507 positions
    # Add our python controller in the scene
    root.addObject( printPosition(MechanicalObject=tetraMesh.Volume, rootnode=root) )

def main():
    import Sofa.Gui
    # Make sure to load all SOFA libraries

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