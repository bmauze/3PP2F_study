######################################################
# Importation of the different required files
######################################################
import Sofa, numpy
from splib.constants import Key
from math import sin, cos, sqrt
from splib.animation import AnimationManager
from designVariable_vOscar import edgesList,positionList,DOF0TransformNode0,DOF1TransformNode1,numberElement
######################################################
# Definition of global variables
######################################################
stepTime=1.0e-1
gravity=-9.81e3
startingTime=0.1
indexPlatform=0
NorP=0
disMaxAct=70
stepDisplacement=0.5e-1
convergenceWaiting=0
toleranceConvergence=1.1e1
recordingBool=0
######################################################
# Definition of the python controller
######################################################
class PCRController(Sofa.PythonScriptController):

    def createGraph(self, node):
        # print('---------- Entering createGraph ----------')
        self.node = node
        # print('---------- Exiting createGraph  ----------')

    # def __init__(self, node):

    def reset(self):
        # print('---------- Entering reset ----------')
        self.time = 0.0
        self.MechPosList=self.node.RPCModel.MechanicalStructure.position
        self.NorP=NorP
        fA = open("forceAct.txt", "w")
        fA.close()
        Ts = open("TimeSim.txt", "w")
        Ts.close()
        self.stepDisplacement=stepDisplacement
        self.convergenceWaiting=convergenceWaiting
        self.recordingBool=0
        self.q1 = 0 # Act1X
        self.q2 = 0 # Act1Y
        self.q3 = 0 # Act2X
        self.q4 = 0 # Act2Y
        self.q5 = 0 # Act3X
        self.q6 = 0 # Act3Y
        self.extf = self.node.RPCModel.ExtF
        self.offsetForce = 1e2
        # print('---------- Exiting reset  ----------')
        return 0

    def onBeginAnimationStep(self, dt):
        self.time += dt
        MechPosList=self.node.RPCModel.MechanicalStructure.position
        # print(self.node.RPCModel.Act1x.displacement)
        if self.convergenceWaiting>toleranceConvergence:

            self.stepDisplacement+=stepDisplacement
            self.node.RPCModel.Act1x.displacement = self.q1*self.NorP*self.stepDisplacement
            MechPosList[1][0] = self.MechPosList[1][0] + self.node.RPCModel.Act1x.displacement
            self.node.RPCModel.Act1y.displacement = self.q2*self.NorP*self.stepDisplacement
            MechPosList[1][1] = self.MechPosList[1][1] + self.node.RPCModel.Act1y.displacement
            self.node.RPCModel.Act2x.displacement = self.q3*self.NorP*self.stepDisplacement
            MechPosList[2][0] = self.MechPosList[2][0] + self.node.RPCModel.Act2x.displacement
            self.node.RPCModel.Act2y.displacement = self.q4*self.NorP*self.stepDisplacement
            MechPosList[2][1] = self.MechPosList[2][1] + self.node.RPCModel.Act2y.displacement
            self.node.RPCModel.Act3x.displacement = self.q5*self.NorP*self.stepDisplacement
            MechPosList[3][0] = self.MechPosList[3][0] + self.node.RPCModel.Act3x.displacement
            self.node.RPCModel.Act3y.displacement = self.q6*self.NorP*self.stepDisplacement
            MechPosList[3][1] = self.MechPosList[3][1] + self.node.RPCModel.Act3y.displacement

            self.node.RPCModel.MechanicalStructure.position=MechPosList
            # print(self.node.RPCModel.MechanicalStructure.position[0])
            self.convergenceWaiting=0
        if self.convergenceWaiting==toleranceConvergence:
            forceActListBuf=[abs(self.node.RPCModel.Act1x.force), abs(self.node.RPCModel.Act1y.force), abs(self.node.RPCModel.Act2x.force), 
                abs(self.node.RPCModel.Act2y.force), abs(self.node.RPCModel.Act3x.force), abs(self.node.RPCModel.Act3y.force)]
            # recording actuation forces
            fA = open("forceAct.txt", "a")
            fA.write(str((forceActListBuf)) + '\n')
            fA.close()
            Ts = open("TimeSim.txt", "a")
            Ts.write(str((self.time)) + '\n')
            Ts.close()
        self.convergenceWaiting+=1
        # print('---------- Exiting onBeginAnimationStep  ----------')
        return 0

    def onKeyPressed(self, key):
        extForListBuf = self.extf.forces;
        print("Key Pressed")
        if key == Key.A: 
            if self.q1 == 1:
                self.q1 = 0
                print("Act1 - Dir X not considered anymore")
            else:
                self.q1 = 1
                print("Act1 - Dir X considered")

        if key == Key.Z: 
            if self.q2 == 1:
                self.q2 = 0
                print("Act1 - Dir Y not considered anymore")
            else:
                self.q2 = 1
                print("Act1 - Dir Y considered")

        if key == Key.E: 
            if self.q3 == 1:
                self.q3 = 0
                print("Act2 - Dir X not considered anymore")
            else:
                self.q3= 1
                print("Act2 - Dir X considered")
                        
        if key == Key.R: 
            if self.q4 == 1:
                self.q4 = 0
                print("Act2 - Dir Y not considered anymore")
            else:
                self.q4 = 1
                print("Act2 - Dir Y considered")

        if key == Key.T: 
            if self.q5 == 1:
                self.q5 = 0
                print("Act3 - Dir X not considered anymore")
            else:
                self.q5 = 1
                print("Act3 - Dir X considered")

        if key == Key.Y: 
            if self.q6 == 1:
                self.q6 = 0
                print("Act3 - Dir Y not considered anymore")
            else:
                self.q6 = 1
                print("Act3 - Dir Y considered")

        if key == Key.plus:      # mouvements suivant l'axe Y
            self.NorP=1 
            print("Positive displacement considered")
        elif key == Key.minus:
            self.NorP=-1 
            print("Negative displacement considered")
        elif key == Key.Q:
            self.NorP=0
            print("No displacement")

        # external force:              
        if key == Key.G: # mouvements suivant l'axe X
            extForListBuf[0][0] = self.extf.forces[0][0] - self.offsetForce;
        elif key == Key.H:
            extForListBuf[0][0] = self.extf.forces[0][0] + self.offsetForce;
        elif key == Key.J:      # mouvements suivant l'axe Y
            extForListBuf[0][1] = self.extf.forces[0][1] + self.offsetForce;
        elif key == Key.K:
            extForListBuf[0][1] = self.extf.forces[0][1] - self.offsetForce;
        elif key == Key.L:      # mouvements suivant l'axe Z
            extForListBuf[0][2] = self.extf.forces[0][2] + self.offsetForce;
        elif key == Key.M:
            extForListBuf[0][2] = self.extf.forces[0][2] - self.offsetForce;
        self.extf.forces = extForListBuf;
        print('Position = ',self.node.RPCModel.MechanicalStructure.position[0])
        print('Force = ' + str(self.extf.forces) + '\n')
        
######################################################
# Definition of the scene
######################################################
def createScene(rootNode):
    print('---------- Entering createScene ----------')
    rootNode.createObject('RequiredPlugin',pluginName='SoftRobots BeamAdapter SofaPython SofaSparseSolver')
    rootNode.createObject('VisualStyle',displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    rootNode.createObject('BackgroundSetting', color=[0.0, 0.0, 0.0, 1.0])
    # rootNode.createObject('FreeMotionAnimationLoop')
    AnimationManager(rootNode)
    rootNode.dt = stepTime
    rootNode.gravity = [0.0, -gravity, 0.0]    

    # Declaration of the use of the inverse computation
    # rootNode.createObject('QPInverseProblemSolver')

    # Creation of the Python controller
    rootNode.createObject('PythonScriptController', classname="PCRController")

    # Creation of the RPC model
    RPCModelNode=rootNode.createChild('RPCModel')
    # Definition of the solvers, the convergence parameters and the contrains correction
    RPCModelNode.createObject('EulerImplicitSolver', rayleighStiffness=0.0, printLog=False, rayleighMass=0.0)
    RPCModelNode.createObject('SparseLDLSolver', name='ldl')
    RPCModelNode.createObject('GenericConstraintCorrection', name='GCC', solverName='ldl')
  
    # Definition of the different points that constitued the robot
    RPCModelNode.createObject('MechanicalObject', name='MechanicalStructure', template='Rigid3d', showObject='true', position=positionList, showObjectScale='0.01')
    # Definition of the existing link between the different points that constituted the robot
    RPCModelNode.createObject('MeshTopology', edges=edgesList)
    # Creation of the beam between the different elements and the relation between the the defined position and the corresponding clamping points
    RPCModelNode.createObject('BeamInterpolation', name='interpolation', crossSectionShape='circular', radius='0.5', innerRadius='0.0', 
        dofsAndBeamsAligned='false', defaultYoungModulus='160e6', DOF0TransformNode0=DOF0TransformNode0, DOF1TransformNode1=DOF1TransformNode1) 
    RPCModelNode.createObject('AdaptiveBeamForceFieldAndMass', massDensity=8e-6, name='LinkForceField', interpolation='@interpolation', reinforceLength=1)

    # Definition of the different fixed and clamping points of the robotic structure
    RPCModelNode.createObject('PartialFixedConstraint', name='ConstraintPlat', indices='0', fixedDirections='0 0 0 0 0 0')
    RPCModelNode.createObject('PartialFixedConstraint', name='ConstraintAct1', indices='1', fixedDirections='1 1 1 1 1 1') 
    RPCModelNode.createObject('PartialFixedConstraint', name='ConstraintAct2', indices=2*numberElement, fixedDirections='1 1 1 1 1 1') 
    RPCModelNode.createObject('PartialFixedConstraint', name='ConstraintAct3', indices=4*numberElement-1, fixedDirections='1 1 1 1 1 1') 
    

    # Definition of the different masses of the different physical component of the robot
    RPCModelNode.createObject('UniformMass', name='MassPlat', indices='0', totalMass='0.004', showAxisSizeFactor='0.001')
    RPCModelNode.createObject('UniformMass', name='MassAct1', indices='1', totalMass='0.0036', showAxisSizeFactor='0.001')
    RPCModelNode.createObject('UniformMass', name='MassAct2', indices=2*numberElement, totalMass='0.0036', showAxisSizeFactor='0.001')
    RPCModelNode.createObject('UniformMass', name='MassAct3', indices=4*numberElement-1, totalMass='0.0036', showAxisSizeFactor='0.001')

    # Actuation variables
    deplacementActionneurx=70     #mm
    deplacementActionneury=70     #mm
    SetpDeplacementActionneur=0.1 #mm 
    forceActionneur=20e3            #(Kg*mm)/s^2
    # Definition of the different actuators
    PosAct1x = RPCModelNode.createObject('SlidingActuator', name='Act1x',template='Rigid3d', indices='1', direction='1 0 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneurx, maxNegativeDisp=deplacementActionneurx)
    PosAct1y = RPCModelNode.createObject('SlidingActuator', name='Act1y',template='Rigid3d', indices='1', direction='0 1 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneury, maxNegativeDisp=deplacementActionneury)
    PosAct2x = RPCModelNode.createObject('SlidingActuator', name='Act2x',template='Rigid3d', indices=2*numberElement, direction='1 0 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneurx, maxNegativeDisp=deplacementActionneurx)
    PosAct2y = RPCModelNode.createObject('SlidingActuator', name='Act2y',template='Rigid3d', indices=2*numberElement, direction='0 1 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneury, maxNegativeDisp=deplacementActionneury)
    PosAct3x = RPCModelNode.createObject('SlidingActuator', name='Act3x',template='Rigid3d', indices=4*numberElement-1, direction='1 0 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneurx, maxNegativeDisp=deplacementActionneurx)
    PosAct3y = RPCModelNode.createObject('SlidingActuator', name='Act3y',template='Rigid3d', indices=4*numberElement-1, direction='0 1 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneury, maxNegativeDisp=deplacementActionneury)

    #Force externe en -z
    extF = RPCModelNode.createObject('ConstantForceField',name='ExtF', indices='0', forces="0 0 0  0 0 0")

    # Monitors management
    RPCModelNode.createObject('Monitor',    name='PlatM', template='Rigid3', listening="1", indices="0", showTrajectories="1", TrajectoriesPrecision="0.01", TrajectoriesColor="0 0 1 1", sizeFactor="1", ExportPositions="true", ExportVelocities="true",ExportForces="true")
    RPCModelNode.createObject('Monitor', name='SldActM1', template='Rigid3', listening="1", indices="1", showTrajectories="0", TrajectoriesPrecision="0.01", TrajectoriesColor="0 0 1 1", sizeFactor="1", ExportPositions="true", ExportVelocities="true",ExportForces="true")
    RPCModelNode.createObject('Monitor', name='SldActM2', template='Rigid3', listening="1", indices=2*numberElement, showTrajectories="0", TrajectoriesPrecision="0.01", TrajectoriesColor="0 0 1 1", sizeFactor="1", ExportPositions="true", ExportVelocities="true",ExportForces="true")
    RPCModelNode.createObject('Monitor', name='SldActM3', template='Rigid3', listening="1", indices=4*numberElement-1, showTrajectories="0", TrajectoriesPrecision="0.01", TrajectoriesColor="0 0 1 1", sizeFactor="1", ExportPositions="true", ExportVelocities="true",ExportForces="true")
    
######################################################
# Visual Definition
######################################################

    VisuRigidNode = RPCModelNode.createChild('Platform')
    VisuRigidNode.createObject('MeshSTLLoader', filename='plataforme.stl', name='loader', translation='0 0 0', rotation='0.0 0.0 180', scale3d='1 1 1')
    VisuRigidNode.createObject('OglModel', src='@loader', name='visuPlatform')
    VisuRigidNode.createObject('RigidMapping', output='@visuPlatform', index='0')
    
    VisuRigidNode1 = RPCModelNode.createChild('Act1')
    VisuRigidNode1.createObject('MeshSTLLoader', filename='base_droite.stl', name='loader', translation='0 0 0', rotation='0.0 0.0 180', scale3d='1 1 1')
    VisuRigidNode1.createObject('OglModel', src='@loader', name='visuBC')
    VisuRigidNode1.createObject('RigidMapping', output='@visuBC', index='1')
    
    VisuRigidNode2 = RPCModelNode.createChild('Act2')
    VisuRigidNode2.createObject('MeshSTLLoader', filename='base_gauche.stl', name='loader', translation='0 0 0', rotation='90 0.0 -90.0', scale3d='1 1 1')
    VisuRigidNode2.createObject('OglModel', src='@loader', name='visuBD')
    VisuRigidNode2.createObject('RigidMapping', output='@visuBD', index=2*numberElement)
    
    VisuRigidNode3 = RPCModelNode.createChild('Act3')
    VisuRigidNode3.createObject('MeshSTLLoader', filename='base_central.stl', name='loader', translation='0 0 0', rotation='0.0 0.0 180.0', scale3d='1 1 1')
    VisuRigidNode3.createObject('OglModel', src='@loader', name='visuBG')
    VisuRigidNode3.createObject('RigidMapping', output='@visuBG', index=4*numberElement-1)
    


    print('---------- Exiting createScene ----------')
    return rootNode