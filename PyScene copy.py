import Sofa
import Sofa.Core
from Sofa.constants import *

class KeyPressedController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)   
        self.listening = True
        self.InterventionalRadiologyController = kwargs.get("IRC")

    def onKeypressedEvent(self, event):

        if event['key'] == '+':
            self.InterventionalRadiologyController.speed.value += 1.0 
            print("fuckyou: ", self.InterventionalRadiologyController.indexFirstNode)
            print("speed: ", self.InterventionalRadiologyController.speed.value)
            print("IfDirty: ", self.InterventionalRadiologyController.speed.isDirty())


        if event['key'] == '-':
            self.InterventionalRadiologyController.speed.value -= 1.0 
            print("speed: ", self.InterventionalRadiologyController.speed.value)
            print("IfDirty: ", self.InterventionalRadiologyController.speed.isDirty())



def createScene(rootNode):

    rootNode.gravity = [0, 0, 0]
    rootNode.bbox = "-10 -10 -10 10 10 10"

    rootNode.addObject('RequiredPlugin', pluginName='BeamAdapter SofaMeshCollision SofaBoundaryCondition SofaConstraint SofaMiscCollision SofaDeformable SofaGeneralLinearSolver SofaImplicitOdeSolver Sofa.Component.Collision.Detection.Algorithm Sofa.Component.IO.Mesh')
    rootNode.addObject('VisualStyle', displayFlags='showAll')
    
    rootNode.addObject('DefaultAnimationLoop')
    #rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('LCPConstraintSolver', mu='0.1', tolerance='1e-10', maxIt='1000', build_lcp='false')
    rootNode.addObject('DefaultPipeline', draw='0', depth='6', verbose='1')
    rootNode.addObject('BruteForceBroadPhase', name='N2')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('LocalMinDistance', contactDistance='1', alarmDistance='5', name='localmindistance', angleCone='0.02')
    rootNode.addObject('DefaultContactManager', name='Response', response='FrictionContactConstraint')


    topoLines = rootNode.addChild('EdgeTopology')
    topoLines.addObject('WireRestShape', name='BeamRestShape', 
                                 straightLength=1000.0, length=1000.0, 
                                 numEdges=200, youngModulus=20000, 
                                 spireDiameter=25, numEdgesCollis=[50,10], 
                                 printLog=True, template='Rigid3d', spireHeight=0.0, 
                                 densityOfBeams=[30,5], youngModulusExtremity=20000)
    topoLines.addObject('EdgeSetTopologyContainer', name='meshLinesBeam')
    topoLines.addObject('EdgeSetTopologyModifier', name='Modifier')
    topoLines.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
    topoLines.addObject('MechanicalObject', name='dofTopo2', template='Rigid3d')


    BeamMechanics = rootNode.addChild('BeamModel')
    BeamMechanics.addObject('EulerImplicitSolver', rayleighStiffness=0.2, printLog=False, rayleighMass=0.1)
    BeamMechanics.addObject('BTDLinearSolver', verification=False, subpartSolve=False, verbose=False)
    BeamMechanics.addObject('RegularGridTopology', name='MeshLines', drawEdges=True, 
                                    nx=60, ny=1, nz=1,
                                    xmax=0.0, xmin=0.0, ymin=0, ymax=0, zmax=0, zmin=0,
                                    p0=[0,0,0])
    BeamMechanics.addObject('MechanicalObject', showIndices=False, name='DOFs Container', template='Rigid3d')
    BeamMechanics.addObject('WireBeamInterpolation', name='BeamInterpolation', WireRestShape='@../EdgeTopology/BeamRestShape', 
                                    radius=0.9, printLog=False)
    BeamMechanics.addObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', massDensity=0.00000155, interpolation='@BeamInterpolation')
    IRC = BeamMechanics.addObject('InterventionalRadiologyController', name='DeployController', template='Rigid3d', instruments='BeamInterpolation', 
                                    startingPos=[0, 0, 0, 0, 0, 0, 1], xtip=[0, 0, 0], printLog=True, 
                                    rotationInstrument=[0, 0, 0], step=0.5, speed=0.5, 
                                    listening=True, controlledInstrument=0)
    BeamMechanics.addObject('FixedConstraint', indices=0, name='FixedConstraint')
    BeamMechanics.addObject('RestShapeSpringsForceField', name="RestSPForceField", points='@DeployController.indexFirstNode', angularStiffness=1e8, stiffness=1e8)

    # rootNode/BeamModelHead
    BeamMechanicsHead = rootNode.addChild('BeamModelHead')
    BeamMechanicsHead.addObject('EulerImplicitSolver', rayleighStiffness=0.2, rayleighMass=0.1, printLog='false')
    BeamMechanicsHead.addObject('BTDLinearSolver', verbose='0')
    
    BeamMechanicsHead.addObject('RegularGridTopology', name='MeshLines',
                            nx=10, ny=1, nz=1,
                            xmax=15.0, xmin=0.0, ymin=0, ymax=0, zmax=0, zmin=0,
                            p0=[0, 0, 0])
    BeamMechanicsHead.addObject('MechanicalObject', template='Rigid3d', name='DOFs')
    BeamMechanicsHead.addObject('MeshTopology', name='lines', lines='0 1 1 2 2 3 3 4 4 5 5 6 6 7 7 8 8 9')
 
    BeamMechanicsHead.addObject('BeamInterpolation', name='BeamInterpolation', radius='0.1')
    BeamMechanicsHead.addObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', computeMass='1', massDensity='10')

    BeamMechanicsHead.addObject("ConstantForceField", name="ConstantForceField", totalForce=[0, -0.1, 0, 0, 0, 0], indices=9, showArrowSize=10)

    attachConstraint = rootNode.addChild('AttachConstraint')
    attachConstraint.addObject('AttachConstraint', twoWay=True, object1="@../BeamModelHead", object2="@../BeamModel", indices1="0", indices2="0", constraintFactor="1")

    rootNode.addObject(KeyPressedController(name="MyController", IRC=IRC))


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


if __name__ == '__main__':
    main()