import Sofa

def createScene(root):
    root.gravity=[0,-9.81,0]
    root.dt=0.05

    # Enable/ disable options in view tab, see doc/components on sofa website
    root.addObject('VisualStyle', displayFlags="showBehaviorModels showVisual") #  showWireframe
    
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