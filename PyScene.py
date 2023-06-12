import sys
import Sofa
import numpy as np 
import pprint

import Sofa.Core
from Sofa.constants import *
from Sofa.Helper import msg_info

functionDict = { 
	# Arrows
	Key.leftarrow : lambda:print("You pressed the left arrow"), 
	Key.rightarrow : lambda:print("You pressed the right arrow"), 
	Key.uparrow : lambda:print("You pressed the up arrow"), 
	Key.downarrow : lambda:print("You pressed the down arrow"), 
	# Special caracters
	Key.space: lambda:print("You pressed the space"), 
	Key.plus: lambda:print("You pressed the plus"), 
	Key.minus: lambda:print("You pressed the minux"), 
	# Letters
	Key.I: lambda:print("You pressed the letter I"),  
	# KeyPad
	Key.KP_0: lambda:print("You pressed the number 0 on the keypad"),
	Key.KP_1: lambda:print("You pressed the number 1 on the keypad"),
	Key.KP_2: lambda:print("You pressed the number 2 on the keypad"),
	Key.KP_3: lambda:print("You pressed the number 3 on the keypad"),
	Key.KP_4: lambda:print("You pressed the number 4 on the keypad"),
	Key.KP_5: lambda:print("You pressed the number 5 on the keypad"),
	Key.KP_6: lambda:print("You pressed the number 6 on the keypad"),
	Key.KP_7: lambda:print("You pressed the number 7 on the keypad"),
	Key.KP_8: lambda:print("You pressed the number 8 on the keypad"),
	Key.KP_9: lambda:print("You pressed the number 9 on the keypad")
		}


class KeyPressedController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)      
        self.listening = True
        self.addData(name='d_speed', type='float', value=0.0, help='speed of the beam')
        # myspeed = Sofa.Core.Data()
        # self.addData(myspeed)


    def onKeypressedEvent(self, event):

        if event['key'] == Key.uparrow:
            print("--------Sofa.Core.COntroller.MEthod--------\n", dir(Sofa.Core.Controller))
            print("--------Sofa.COre.MEthod--------\n", dir(Sofa.Core))
            print("--------SOfa.Method--------\n", dir(Sofa))
            print("--------Data.Method--------\n", dir(self.getData('d_speed')))
            print("--------DataEngine.Method--------\n", dir(Sofa.Core.DataEngine))
            print("--------testMEssage--------\n", self.getContext())
            
            print("value: ",self.getData('d_speed').value)
            print("IsDirty: ", self.getData('d_speed').isDirty())
            print("IsSet: ", self.getData('d_speed').isSet())


        if event['key'] == Key.downarrow:
            self.getData('speed').value -= 1.0

    def onKeyreleasedEvent(self, event):
        print("You released a key!")

class MyDataEngine(Sofa.Core.DataEngine):

    def __init__(self, *args, **kwargs):
        Sofa.Core.DataEngine.__init__(self, *args, **kwargs)
        pass

    def init(self):
        pass

    def update():
        # Function called anytime an output is accessed while the component
        # is dirty (input has changed)
        msg_info('Not implemented yet')
        pass


def createScene(rootNode):

    rootNode.addObject('RequiredPlugin', pluginName='BeamAdapter SofaMeshCollision SofaBoundaryCondition SofaConstraint SofaMiscCollision SofaDeformable SofaGeneralLinearSolver SofaImplicitOdeSolver Sofa.Component.Collision.Detection.Algorithm Sofa.Component.IO.Mesh')
    rootNode.addObject('VisualStyle', displayFlags='hideVisualModels hideBehaviorModels showCollisionModels hideMappings showInteractionForceFields')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('LCPConstraintSolver', mu='0.1', tolerance='1e-10', maxIt='1000', build_lcp='false')
    rootNode.addObject('DefaultPipeline', draw='0', depth='6', verbose='1')
    rootNode.addObject('BruteForceBroadPhase', name='N2')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('LocalMinDistance', contactDistance='1', alarmDistance='5', name='localmindistance', angleCone='0.02')
    rootNode.addObject('DefaultContactManager', name='Response', response='FrictionContactConstraint')


    topoLines = rootNode.addChild('EdgeTopology')
    topoLines.addObject('WireRestShape', name='BeamRestShape', 
                                 straightLength=980.0, length=1000.0, 
                                 numEdges=200, youngModulus=20000, 
                                 spireDiameter=25, numEdgesCollis=[50,10], 
                                 printLog=True, template='Rigid3d', spireHeight=0.0, 
                                 densityOfBeams=[30,5], youngModulusExtremity=20000)
    topoLines.addObject('EdgeSetTopologyContainer', name='meshLines')
    topoLines.addObject('EdgeSetTopologyModifier', name='Modifier')
    topoLines.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    topoLines.addObject('MechanicalObject', name='dofTopo2', template='Rigid3d')

    BeamMechanics = rootNode.addChild('BeamModel')

    MyControllerObject = KeyPressedController(name="MyController")
    BeamMechanics.addObject(MyControllerObject)

    BeamMechanics.addObject(MyDataEngine(name="MyDataEngine") )

    BeamMechanics.addObject('EulerImplicitSolver', rayleighStiffness=0.2, rayleighMass=0.1)
    BeamMechanics.addObject('BTDLinearSolver', verification=False, subpartSolve=False, verbose=False)
    BeamMechanics.addObject('RegularGridTopology', name='MeshLines', 
                                    nx=60, ny=1, nz=1,
                                    xmax=0.0, xmin=0.0, ymin=0, ymax=0, zmax=0, zmin=0,
                                    p0=[0,0,0])
    BeamMechanics.addObject('MechanicalObject', showIndices=False, name='DOFs', template='Rigid3d', ry=-90)
    BeamMechanics.addObject('WireBeamInterpolation', name='BeamInterpolation', WireRestShape='@../EdgeTopology/BeamRestShape', 
                                    radius=0.9, printLog=False)
    BeamMechanics.addObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', massDensity=0.00000155, interpolation='@BeamInterpolation')
    BeamMechanics.addObject('InterventionalRadiologyController', name='DeployController', template='Rigid3d', instruments='BeamInterpolation', 
                                    startingPos=[0, 0, 0, 0, 0, 0, 1], xtip=[0, 0, 0], printLog=True, 
                                    rotationInstrument=[0, 0, 0], step=5., speed='@MyController.d_speed', 
                                    listening='@MyController.listening', controlledInstrument=0)
    BeamMechanics.addObject('LinearSolverConstraintCorrection', wire_optimization='true', printLog=False)
    BeamMechanics.addObject('FixedConstraint', indices=0, name='FixedConstraint')
    BeamMechanics.addObject('RestShapeSpringsForceField', points='@DeployController.indexFirstNode', angularStiffness=1e8, stiffness=1e8)
    

    BeamCollis = BeamMechanics.addChild('CollisionModel')
    BeamCollis.activated = True
    BeamCollis.addObject('EdgeSetTopologyContainer', name='collisEdgeSet')
    BeamCollis.addObject('EdgeSetTopologyModifier', name='colliseEdgeModifier')
    BeamCollis.addObject('MechanicalObject', name='CollisionDOFs')
    BeamCollis.addObject('MultiAdaptiveBeamMapping', controller='../DeployController', useCurvAbs=True, printLog=False, name='collisMap')
    BeamCollis.addObject('LineCollisionModel', proximity=0.0)
    BeamCollis.addObject('PointCollisionModel', proximity=0.0)

    # Carotids = rootNode.addChild('Carotids')
    # Carotids.addObject('MeshSTLLoader', filename='withoutPlanewithwall.stl', flipNormals=False, triangulate=True, name='meshLoader', rotation=[10.0, 0.0, -90.0])
    # Carotids.addObject('MeshTopology', position='@meshLoader.position', triangles='@meshLoader.triangles')
    # Carotids.addObject('MechanicalObject', position=[0,0,400], scale=1, name='DOFs1', ry=90)
    # Carotids.addObject('TriangleCollisionModel', moving=False, simulated=False)
    # Carotids.addObject('LineCollisionModel', moving=False, simulated=False)



def main():
    import SofaRuntime
    import Sofa.Gui

    root = Sofa.Core.Node('root')
    createScene(root)
    Sofa.Simulation.init(root)

    Sofa.Gui.GUIManager.Init('myscene', 'qglviewer')
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    Sofa.Gui.GUIManager.closeGUI()


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()