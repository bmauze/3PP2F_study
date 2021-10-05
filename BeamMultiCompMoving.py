import Sofa
from splib.constants import Key
# mm and kg units
# BEAM_LENGTH = 100  # in mm
# BEAM_RADIUS = 0.5  # in mm
# BEAM_DENSITY = 6.45e-6  # in kg/mm^3
# YOUNG_MODULUS = 160e6  # 160 GPa = 160e9 Pa = 160e9 N/(m.s^2) = 160e6 N/(mm.s^2)
# ELEMENT_NUMBER = 10
# GRAVITATIONAL_ACCELERATION = 9806.65  # in mm/s^2
# STEP_TIME = 0.001  # in s
# LOAD_MASS = 0.5  # in kg

# SI Units
BEAM_LENGTH = 0.1  # in m
BEAM_RADIUS = 0.5e-3  # in m
BEAM_DENSITY = 6450  # in kg/m^3
YOUNG_MODULUS = 160e9  # in Pa
ELEMENT_COUNT = 5
GRAVITATIONAL_ACCELERATION = 9.80665  # in m/s^2
STEP_TIME = 0.01  # in s
LOAD_MASS = 0.1  # in kg

# OpenGL
SLICE_COUNT = 100
SECTOR_COUNT = 5

def createScene(rootNode):
    print('---------- Entering createScene ----------')

    rootNode.createObject('RequiredPlugin', name='BeamAdapter')
    rootNode.createObject('RequiredPlugin', name='SofaPython')
    rootNode.createObject('RequiredPlugin', name='SofaSparseSolver')

    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')

    rootNode.createObject('BackgroundSetting', color=[0.0, 0.0, 0.0, 1.0])

    rootNode.dt = STEP_TIME
    rootNode.gravity = [0.0, -GRAVITATIONAL_ACCELERATION, 0.0]
    rootNode.createObject('GenericConstraintSolver', name="genericConstraintSolver1", maxIterations="10000", tolerance="1e-12")
    rootNode.createObject('DefaultPipeline', verbose="0")
    rootNode.createObject('BruteForceDetection', name="N2")
    rootNode.createObject('DefaultContactManager', response="FrictionContact", responseParams="mu=0.6")
    rootNode.createObject('LocalMinDistance', name="Proximity", alarmDistance="0.05", contactDistance="0.001", angleCone="0.00")
    rootNode.createObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    rootNode.createObject('DefaultPipeline', verbose="0")
    rootNode.createObject('BruteForceDetection', name="N2")
    rootNode.createObject('DefaultPipeline', verbose="0")
    rootNode.createObject('PythonScriptController', classname="Controller")

    ##### Compute the beam topology using python
    edgesList = []
    for i in range(0, ELEMENT_COUNT):
        edgesList.append([i, i + 1])

    print("edgesList = ", edgesList)

    positionsList = []
    for i in range(0, ELEMENT_COUNT + 1):
        dx = BEAM_LENGTH / ELEMENT_COUNT
        positionsList.append([dx * i, 0.0, 0.0, 0.0, 0.0, 0.0, 1])

    print("positionsList = ", positionsList)

    wall = rootNode.createChild('wall')
    wall.createObject('EulerImplicitSolver', name="odesolverwall", rayleighStiffness="0.1", rayleighMass="0.1")
    wall.createObject('SparseLDLSolver')
    wall.createObject('MechanicalObject', name='wallFrame', template="Rigid3", position=['0.015 -0.05 0.0  0.0 0.0 0.707 0.707','0.015 0.0 0.0  0.0 0.0 0.707 0.707','0.015 0.05 0.0  0.0 0.0 0.707 0.707'])
    wall.createObject('MeshTopology', edges=['0 1','1 2'])
    # beam.createObject('BeamInterpolation', name='interpolation', crossSectionShape='rectangular', lengthY=2*BEAM_RADIUS, lengthZ=2*BEAM_RADIUS, defaultYoungModulus=YOUNG_MODULUS)
    wall.createObject('BeamInterpolation', name='wallinter', crossSectionShape='circular', radius=BEAM_RADIUS,
                      innerRadius=0.0, defaultYoungModulus=YOUNG_MODULUS, dofsAndBeamsAligned=1)
    wall.createObject('AdaptiveBeamForceFieldAndMass', name='wallForceField', computeMass=1, massDensity=BEAM_DENSITY)
    wall.createObject('PartialFixedConstraint', name='clampingProxEnd', indices=['0','1','2'], fixedDirections='0 1 1 1 1 1')

    ##### Beam model
    beam = rootNode.createChild('beam')
    beam.createObject('EulerImplicitSolver', name="odesolverwall", rayleighStiffness="0.1", rayleighMass="0.1")
    beam.createObject('SparseLDLSolver')
    beam.createObject('MechanicalObject', name='frame', template="Rigid3", position=positionsList)
    beam.createObject('MeshTopology', edges=edgesList)
    # beam.createObject('BeamInterpolation', name='interpolation', crossSectionShape='rectangular', lengthY=2*BEAM_RADIUS, lengthZ=2*BEAM_RADIUS, defaultYoungModulus=YOUNG_MODULUS)
    beam.createObject('BeamInterpolation', name='interpolation', crossSectionShape='circular', radius=BEAM_RADIUS,
                      innerRadius=0.0, defaultYoungModulus=YOUNG_MODULUS, DOF0TransformNode0=['0.0 0.0 0.0  0.0 0.0 0.0 0.0','0.0 0.0 0.0  0.0 0.0 0.0 0.0','0.0 0.0 0.0  0.0 0.0 0.0 0.0'],
                      DOF1TransformNode1=['0.0 0.0 0.0  0.0 0.0 0.0 0.0','0.0 0.0 0.0  0.0 0.0 0.0 0.0','0.0 0.0 0.0  0.0 0.0 0.0 0.0'], dofsAndBeamsAligned=1.0)
    beam.createObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', computeMass=1, massDensity=BEAM_DENSITY)

    ##### Clamping
    # beam.createObject('FixedConstraint', name='clamping', indices='0')
    beam.createObject('PartialFixedConstraint', name='clampingProxEnd', indices='0', fixedDirections='0 1 1 1 1 1')
    beam.createObject('FixedConstraint', name='clampingDisEnd', indices=ELEMENT_COUNT)

    rootNode.createObject('AttachConstraint', name='constforPal', object1="@wall", object2="@beam", indices1="0", indices2="0", constraintFactor="1",twoWays='1')

    print('---------- Exiting createScene ----------')

    return rootNode


class Controller(Sofa.PythonScriptController):

    def createGraph(self, node):
        print('---------- Entering createGraph ----------')
        self.node = node
        print('---------- Exiting createGraph ----------')

    def reset(self):
        print('---------- Entering reset ----------')
        self.time = 0.0
        print('---------- Exiting reset ----------')
        return 0

    def onBeginAnimationStep(self, dt):
        return 0

    def onEndAnimationStep(self, dt):
        return 0

    def onKeyPressed(self, key):
        print("Key Pressed")
        if key == Key.U: # mouvements suivant l'axe X
            self.node.wall.wallFrame.rest_position=self.node.wall.wallFrame.position
        return 0

