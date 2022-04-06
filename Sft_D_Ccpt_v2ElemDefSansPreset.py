import Sofa
import numpy
from splib.constants import Key
from math import sin, cos, sqrt
from stlib.physics.constraints import *
from stlib.physics.deformable import ElasticMaterialObject
from stlib.physics.mixedmaterial import Rigidify
from splib.objectmodel import SofaPrefab, SofaObject, setData

PI = 3.14159265359
meshRobot='bloc_v2_mesh1.vtk'

def createScene(rootNode):
    # Root node
    rootNode.findData('dt').value=0.05
    rootNode.findData('gravity').value='0 0 -9.81'
    rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showCollision showVisualModels showForceFields showInteractionForceFields hideCollisionModels hideBoundingCollisionModels hideWireframe')
    rootNode.createObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    #Required plugin
    rootNode.createObject('RequiredPlugin', pluginName=' SoftRobots SofaPython SofaPreconditioner SofaSparseSolver')
    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('DefaultPipeline', verbose='0')
    rootNode.createObject('BruteForceDetection', name='N2')
    rootNode.createObject('DefaultContactManager', response='FrictionContact')
    rootNode.createObject('LocalMinDistance', name="Proximity", alarmDistance='3', contactDistance='0.5')
    rootNode.createObject('QPInverseProblemSolver', name="QP", printLog='0')
    StructMeca = rootNode.createChild('StructMeca')
    StructMecaDefElem = ElasticMaterialObject(StructMeca, volumeMeshFileName=meshRobot, translation= [0.0, 0.0, 0.0], rotation=[0.0, 0.0, 0.0],
                                        youngModulus=180.0, poissonRatio=0.45, totalMass=0.050)
    StructMecaDefElem.init()
# Box management
    FixedBox(StructMeca.ElasticMaterialObject, atPositions="-2.51 -0.01 -11.01   2.51 1.01 -10.00", doVisualization="true")
    commandBox=StructMeca.ElasticMaterialObject.createObject('BoxROI', name="boxROI1", box="-2.51 -0.01 0.701    2.51 1.01  1.01", drawBoxes="true")
    commandBox.init()
# Rigidification management
    # groupIndicesTest=[[1, 2], [4, 5], [26, 27, 28, 29, 30, 31], [66,67,68,69]]
    rigidifiedstruct = Rigidify( StructMeca, StructMecaDefElem , groupIndices=StructMeca.ElasticMaterialObject.boxROI1.indices, name="RigidifiedStructure")
    # rigidifiedstruct = Rigidify( StructMeca, StructMecaDefElem , groupIndices=groupIndicesTest, name="RigidifiedStructure")
    rigidifiedstruct.DeformableParts.createObject("UncoupledConstraintCorrection")
    rigidifiedstruct.RigidParts.RigidifiedParticules.createObject("UncoupledConstraintCorrection")
    rigidifiedstruct.RigidParts.createObject('PartialFixedConstraint', name="PFC1", indices="@dofs.indices",fixedDirections='0 0 0 0 0 0')
    # setData(rigidifiedstruct.RigidParts.dofs, showObject=True, showObjectScale=1, drawMode=2)
    setData(rigidifiedstruct.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=0.1, drawMode=1, showColor=[1., 1., 0., 1.])
    setData(rigidifiedstruct.DeformableParts.dofs, showObject=True, showObjectScale=0.1, drawMode=2)
    # StructMeca.createObject('SparseLDLSolver', name="preconditioner")
    # StructMeca.createObject('LinearSolverConstraintCorrection', solverName="preconditioner")
    # StructMeca.createObject('ShewchukPCGLinearSolver', iterations="1", name="linearsolver", tolerance="1e-5", preconditioners="preconditioner", use_precond="true", update_step="1")
    
    simulationNode = rootNode.createChild("Simulation")
    simulationNode.createObject("EulerImplicitSolver")
    simulationNode.createObject("CGLinearSolver")
    simulationNode.addChild(rigidifiedstruct)
    StructMeca.RigidifiedStructure.RigidParts.RigidifiedParticules.createObject('ConstantForceField', name='cff', forces='10.0 0 0 0 0 0', indices="@ElasticMaterialObject/boxROI1.indices", arrowSizeCoef='0.03')
    return rootNode