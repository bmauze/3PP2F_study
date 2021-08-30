######################################################
# Importation of the different required files
######################################################
import Sofa, numpy
from splib.constants import Key
from math import sin, cos, sqrt
from splib.animation import AnimationManager
# Importation of the different design's variables
# Discretization des poutres pour une division de 7 
positionList=  [[0.003423230215944049, 0.006022613451855615, 0, -3.265265483357694e-05, 3.183590147299464e-05, -7.225301700406276e-06, 0.9999999989340372], [0.0, -74.93, -50.0, 0.0, 0.0, 0.0, 1.0], [64.89, 37.46, -50.0, 0.0, 0.0, 0.0, 1.0], [-64.89, 37.46, -50.0, 0.0, 0.0, 0.0, 1.0],
                [3.7732461203708056, -67.28303530596328, -42.34837078986425, 0.30932849931590245, -0.22527266388142683, 0.5438871253707012, 0.7468299011654077], [8.344987009107083, -60.139134008050945, -35.20521051629461, 0.3307958157176193, -0.19235319440438783, 0.4644246791125793, 0.798676464118807], [14.596352281645144, -53.302665290395545, -28.36966739356365, 0.33962401637042583, -0.17629059500388733, 0.4256520753593123, 0.8199984538765341], [21.40759234878376, -46.568674099492455, -21.63666332111222, 0.3396240163704257, -0.17629059500388738, 0.42565207535931243, 0.819998453876534], [27.65895762132179, -39.732205381837026, -14.801120198381243, 0.33079581571662275, -0.1923531944038326, 0.46442467911444774, 0.798676464118267], [32.2306985100581, -32.58830408392475, -7.657959924811641, 0.3093284993159024, -0.22527266388142686, 0.5438871253707012, 0.7468299011654076],
                [-3.77270549208419, -67.2830704858334, -42.34802492876199, 0.22527352452473096, -0.3093403968738054, 0.746838128166873, 0.5438687050441157], [-8.344745221409623, -60.139304422853414, -35.204538162283015, 0.19236464249531754, -0.3308080338628193, 0.7986722047171432, 0.4644185595577788], [-14.596100382538518, -53.30306647233731, -28.36859446316974, 0.1763067869315712, -0.33963700093284854, 0.8199899762942128, 0.4256513400145971], [-21.407227354268556, -46.569337892642494, -21.63516511984502, 0.17630678693095347, -0.3396370009340422, 0.8199899762958599, 0.42565134001072713], [-27.65858251539746, -39.73309994212639, -14.799221420731737, 0.19236464249543364, -0.3308080338625946, 0.7986722047168229, 0.4644185595584414], [-32.230622244722916, -32.58933387914642, -7.655734654252784, 0.22527352452477972, -0.30934039687372344, 0.7468381281667408, 0.5438687050443237],
                [56.3629182704205, 36.890830355727005, -42.34057277432018, 0.3497946731795367, 0.15523949087141084, 0.9187100242661151, -0.09755141414741521], [47.893265965437756, 37.2793937813657, -35.19845040935193, 0.3320572771523423, 0.19029883347565446, 0.923859529208628, -0.002808728963425641], [38.850359970108784, 39.27612031374339, -28.363020564199054, 0.3225793121996883, 0.20597771931060255, 0.9229298700850647, 0.04142730250866273], [29.616369411331128, 41.808901215034105, -21.629821558984666, 0.32257931219967134, 0.20597771931097464, 0.9229298700849361, 0.041427302509807216], [20.573463416002213, 43.805627747411855, -14.794391713831834, 0.33205727715233585, 0.1902988334757654, 0.9238595292086084, -0.002808728963105643], [12.103811111019523, 44.19419117305059, -7.652269348863598, 0.34979467317954555, 0.15523949087133634, 0.9187100242661038, -0.09755141414760829], 
                [60.13496513373158, 30.361616289557425, -42.340542609414356, 0.38057548392377566, 0.04040321050673016, 0.8444157479296955, -0.37482252635782926], [56.23597388163039, 22.83306703583144, -35.19826419482315, 0.38275011948220167, 0.0011859631861161784, 0.8015169172844444, -0.4594252614240127], [53.442695997852006, 14.0043241697263, -28.36256176141344, 0.382384824457574, -0.01712547156602245, 0.7786004148555119, -0.4972624641323901], [51.01798923684777, 4.742183432828078, -21.629051321730802, 0.3823848244571643, -0.017125471565636364, 0.7786004148565946, -0.4972624641310228], [48.22471135306936, -4.086559433277073, -14.793348888321065, 0.38275011948220167, 0.0011859631861162146, 0.8015169172844445, -0.4594252614240126], [44.325720100968205, -11.615108687003048, -7.65107047372989, 0.3805754839238175, 0.04040321050664909, 0.8444157479295875, -0.37482252635803887],
                [-60.144152829233704, 30.365893727074006, -42.34433884256346, -0.04039729666920466, -0.38054510085767396, -0.37482107606643367, 0.8444303675090347], [-56.244319295557176, 22.836140523711983, -35.201794271313915, -0.0011722655664491576, -0.38271861793983575, -0.4594397750452322, 0.8015236605254082], [-53.45060048605298, 14.005930772355171, -28.366086508759743, 0.017143066921053437, -0.38235258478727396, -0.4972844785783507, 0.7786018003627326], [-51.02558658460622, 4.742235505000095, -21.632657682437383, 0.017143066921053354, -0.38235258478727385, -0.4972844785783505, 0.7786018003627329], [-48.23186777510201, -4.087974246356733, -14.796949919883218, -0.0011722655661192936, -0.3827186179407977, -0.4594397750470731, 0.8015236605238945], [-44.33203424142552, -11.617727449718748, -7.654405348633672, -0.04039729666906909, -0.3805451008578928, -0.37482107606691756, 0.844430367508728], 
                [-56.37150428253573, 36.89621650636697, -42.344038396260586, -0.15523885998353304, -0.3497718028499731, -0.09754080081078705, 0.9187199652011939], [-47.90035726770661, 37.28496463681254, -35.20133781931453, -0.1902995923060326, -0.3320384634347836, -0.0027965006783653, 0.9238661718842082], [-38.85615624611977, 39.28214498603925, -28.36551980872209, -0.20597985326891158, -0.3225622037119929, 0.041442233633267864, 0.9229347030290205], [-29.620937222280336, 41.81546940819301, -21.631995986914184, -0.2059798532689635, -0.32256220370842154, 0.04144223363213547, 0.9229347030303079], [-20.57673620069348, 43.81264975741971, -14.796177976321726, -0.19029959230568638, -0.3320384634317947, -0.002796500679956999, 0.9238661718853489], [-12.105589185864382, 44.20139788786531, -7.6534773993756975, -0.15523885998353307, -0.349771802849973, -0.097540800810787, 0.9187199652011939]]
edgesList= [[1, 4], [4, 5], [5, 6], [6, 7], [7, 8], [8, 9], [9, 0], 
            [1, 10], [10, 11], [11, 12], [12, 13], [13, 14], [14, 15], [15, 0], 
            [2, 16], [16, 17], [17, 18], [18, 19], [19, 20], [20, 21], [21, 0], 
            [2, 22], [22, 23], [23, 24], [24, 25], [25, 26], [26, 27], [27, 0], 
            [3, 28], [28, 29], [29, 30], [30, 31], [31, 32], [32, 33], [33, 0], 
            [3, 34], [34, 35], [35, 36], [36, 37], [37, 38], [38, 39], [39, 0]]
DOF0TransformNode0 = [[2.0002, 2.95826e-05, -7.14203e-05, 0.2706, -0.2706, 0.6533, 0.6533], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], 
                      [-2.0002, -2.95826e-05, 7.14203e-05, 0.2706, -0.2706, 0.6533, 0.6533], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1],
                      [-0.999832, 1.72968, -3.41732e-05, -0.3696, -0.099, -0.8924, 0.2391], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1],
                      [0.999832, -1.72968, 3.41732e-05, -0.3696, -0.099, -0.8924, 0.2391], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1],
                      [-0.999975, -1.72998, -3.40889e-05, -0.099, -0.3696, -0.2391, 0.8924], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1],
                      [0.999975, 1.72998, 3.40889e-05, -0.099, -0.3696, -0.2391, 0.8924], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1]]
DOF1TransformNode1 = [[0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [34.0031, -24.932, -0.0021044, 0.2706, -0.2706, 0.6533, 0.6533],
                      [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [-34.0038, -24.933, 0.000323891, 0.2706, -0.2706, 0.6533, 0.6533],
                      [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [4.58958, 41.9025, 0.00100722, 0.3696, 0.099, 0.8924, -0.2391], 
                      [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [38.5839, -16.9767, 0.00216791, 0.3696, 0.099, 0.8924, -0.2391], 
                      [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [-38.5887, -16.9801, -0.000466096, -0.099, -0.3696, -0.2391, 0.8924], 
                      [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [0, 0, 0, 0, 0, 0, 1], [-4.58953, 41.9094, 0.000694121, -0.099, -0.3696, -0.2391, 0.8924]]
######################################################
# Definition of global variables
######################################################
stepTime=1.0e-1
gravity=0.0#-9.81e3
startingTime=0.1
indexPlatform=0
RouT=0
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
        self.RouT=RouT
        fA = open("forceAct.txt", "w")
        fA.close()
        Ts = open("TimeSim.txt", "w")
        Ts.close()
        self.stepDisplacement=stepDisplacement
        self.convergenceWaiting=convergenceWaiting
        self.recordingBool=0
        self.KxT=0
        self.KyT=0
        self.KzT=0
        self.KxR=0
        self.KyR=0
        self.KzR=0
        # print('---------- Exiting reset  ----------')
        return 0

    def onBeginAnimationStep(self, dt):
        self.time += dt
        MechPosList=self.node.RPCModel.MechanicalStructure.position
        if self.convergenceWaiting>toleranceConvergence:
            print(self.RouT,self.KzT,stepDisplacement)
            self.stepDisplacement+=stepDisplacement

            MechPosList[indexPlatform][0] = self.MechPosList[indexPlatform][0] + self.KxT*self.stepDisplacement
            MechPosList[indexPlatform][1] = self.MechPosList[indexPlatform][1] + self.KyT*self.stepDisplacement
            MechPosList[indexPlatform][2] = self.MechPosList[indexPlatform][2] + self.KzT*self.stepDisplacement
            MechPosList[indexPlatform][3] = self.MechPosList[indexPlatform][3] + self.KxR*self.stepDisplacement*1e-1
            MechPosList[indexPlatform][4] = self.MechPosList[indexPlatform][4] + self.KyR*self.stepDisplacement*1e-1
            MechPosList[indexPlatform][5] = self.MechPosList[indexPlatform][5] + self.KzR*self.stepDisplacement*1e-1

            self.node.RPCModel.MechanicalStructure.position=MechPosList
            print(self.node.RPCModel.MechanicalStructure.position[0])
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
        print("Key Pressed")
        if key == Key.leftarrow: # mouvements suivant l'axe X
            if self.RouT == 1:
                self.KxT=-1
                self.KxR=0
                print("X negative translation")
            else:
                self.KxR=-1
                self.KxT=0
                print("X negative rotation")

        elif key == Key.rightarrow:
            if self.RouT == 1:
                self.KxT=1
                self.KxR=0
                print("X positive translation")
            else:
                self.KxR=1
                self.KxT=0
                print("X positive rotation")
                       
        if key == Key.uparrow:   # mouvements suivant l'axe Z
            if self.RouT == 1:
                self.KyT=1
                self.KyR=0
                print("Y positive translation")
            else:
                self.KyR=1
                self.KyT=0
                print("Y positive rotation")
        elif key == Key.downarrow:
            if self.RouT == 1:
                self.KyT=-1
                self.KyR=0
                print("Y negative translation")
            else:
                self.KyR=-1
                self.KyT=0
                print("Y negative rotation")

        if key == Key.plus:      # mouvements suivant l'axe Y
            if self.RouT == 1:
                self.KzT=1
                self.KzR=0
                print("Z positive translation")
            else:
                self.KzR=1
                self.KzT=0
                print("Z positive rotation")
        elif key == Key.minus:
            if self.RouT == 1:
                self.KzT=-1
                self.KzR=0
                print("Z negative translation")
            else:
                self.KzR=-1
                self.KzT=0
                print("Z negaive rotation")

        if key == Key.T:      # mouvements suivant l'axe Y
            self.RouT=1 # Translation
            print("Translations considered")
        elif key == Key.R:
            self.RouT=0 # Rotation
            print("Rotation considered")
        
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
    RPCModelNode.createObject('PartialFixedConstraint', name='ConstraintPlat', indices='0', fixedDirections='1 1 1 1 1 1')
    RPCModelNode.createObject('PartialFixedConstraint', name='ConstraintAct1', indices='1', fixedDirections='0 0 1 1 1 1') 
    RPCModelNode.createObject('PartialFixedConstraint', name='ConstraintAct2', indices='2', fixedDirections='0 0 1 1 1 1') 
    RPCModelNode.createObject('PartialFixedConstraint', name='ConstraintAct3', indices='3', fixedDirections='0 0 1 1 1 1') 
    

    # Definition of the different masses of the different physical component of the robot
    RPCModelNode.createObject('UniformMass', name='MassPlat', indices='0', totalMass='0.004', showAxisSizeFactor='0.001')
    RPCModelNode.createObject('UniformMass', name='MassAct1', indices='1', totalMass='0.0036', showAxisSizeFactor='0.001')
    RPCModelNode.createObject('UniformMass', name='MassAct2', indices='2', totalMass='0.0036', showAxisSizeFactor='0.001')
    RPCModelNode.createObject('UniformMass', name='MassAct3', indices='3', totalMass='0.0036', showAxisSizeFactor='0.001')

    # Actuation variables
    deplacementActionneurx=70     #mm
    deplacementActionneury=70     #mm
    SetpDeplacementActionneur=0.1 #mm
    forceActionneur=20e3            #(Kg*mm)/s^2
    # Definition of the different actuators
    PosAct1x = RPCModelNode.createObject('SlidingActuator', name='Act1x',template='Rigid3d', indices='1', direction='1 0 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneurx, maxNegativeDisp=deplacementActionneurx)
    PosAct1y = RPCModelNode.createObject('SlidingActuator', name='Act1y',template='Rigid3d', indices='1', direction='0 1 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneury, maxNegativeDisp=deplacementActionneury)
    PosAct2x = RPCModelNode.createObject('SlidingActuator', name='Act2x',template='Rigid3d', indices='2', direction='1 0 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneurx, maxNegativeDisp=deplacementActionneurx)
    PosAct2y = RPCModelNode.createObject('SlidingActuator', name='Act2y',template='Rigid3d', indices='2', direction='0 1 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneury, maxNegativeDisp=deplacementActionneury)
    PosAct3x = RPCModelNode.createObject('SlidingActuator', name='Act3x',template='Rigid3d', indices='3', direction='1 0 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneurx, maxNegativeDisp=deplacementActionneurx)
    PosAct3y = RPCModelNode.createObject('SlidingActuator', name='Act3y',template='Rigid3d', indices='3', direction='0 1 0 0 0 0', maxForce=forceActionneur, minForce=-forceActionneur, maxDispVariation=SetpDeplacementActionneur, maxPositiveDisp=deplacementActionneury, maxNegativeDisp=deplacementActionneury)

    # Monitors management
    RPCModelNode.createObject('Monitor',    name='PlatM', template='Rigid3', listening="1", indices="0", showTrajectories="1", TrajectoriesPrecision="0.01", TrajectoriesColor="0 0 1 1", sizeFactor="1", ExportPositions="true", ExportVelocities="true",ExportForces="true")
    RPCModelNode.createObject('Monitor', name='SldActM1', template='Rigid3', listening="1", indices="1", showTrajectories="0", TrajectoriesPrecision="0.01", TrajectoriesColor="0 0 1 1", sizeFactor="1", ExportPositions="true", ExportVelocities="true",ExportForces="true")
    RPCModelNode.createObject('Monitor', name='SldActM2', template='Rigid3', listening="1", indices='2', showTrajectories="0", TrajectoriesPrecision="0.01", TrajectoriesColor="0 0 1 1", sizeFactor="1", ExportPositions="true", ExportVelocities="true",ExportForces="true")
    RPCModelNode.createObject('Monitor', name='SldActM3', template='Rigid3', listening="1", indices='3', showTrajectories="0", TrajectoriesPrecision="0.01", TrajectoriesColor="0 0 1 1", sizeFactor="1", ExportPositions="true", ExportVelocities="true",ExportForces="true")
     
######################################################
# Visual Definition
######################################################

    VisuRigidNode = RPCModelNode.createChild('Platform')
    VisuRigidNode.createObject('MeshSTLLoader', filename='plataforme.stl', name='loader', translation='0 0 0', rotation='0.0 0.0 180', scale3d='1 1 1')
    VisuRigidNode.createObject('OglModel', src='@loader', name='visuPlatform')
    VisuRigidNode.createObject('RigidMapping', output='@visuPlatform', index='0')
    
    VisuRigidNode1 = RPCModelNode.createChild('Act1')
    VisuRigidNode1.createObject('MeshSTLLoader', filename='base_central.stl', name='loader', translation='0 0 0', rotation='0.0 0.0 180', scale3d='1 1 1')
    VisuRigidNode1.createObject('OglModel', src='@loader', name='visuBC')
    VisuRigidNode1.createObject('RigidMapping', output='@visuBC', index='1')
    
    VisuRigidNode2 = RPCModelNode.createChild('Act2')
    VisuRigidNode2.createObject('MeshSTLLoader', filename='base_droite.stl', name='loader', translation='0 0 0', rotation='0.0 0.0 180.0', scale3d='1 1 1')
    VisuRigidNode2.createObject('OglModel', src='@loader', name='visuBD')
    VisuRigidNode2.createObject('RigidMapping', output='@visuBD', index='2')
    
    VisuRigidNode3 = RPCModelNode.createChild('Act3')
    VisuRigidNode3.createObject('MeshSTLLoader', filename='base_gauche.stl', name='loader', translation='0 0 0', rotation='90 0.0 -90.0', scale3d='1 1 1')
    VisuRigidNode3.createObject('OglModel', src='@loader', name='visuBG')
    VisuRigidNode3.createObject('RigidMapping', output='@visuBG', index='3')


    print('---------- Exiting createScene ----------')
    return rootNode