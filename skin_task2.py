#PROVA PELLE 2.0 CON DIVERSE STIFFNESS E ALTEZZA DIVERSA (CAPSULA)

# Required import for python
import Sofa
import numpy as np
import Sofa.SofaDeformable
import suture_models
#from goto import goto, label
import subprocess
import read_Files
import time

# data
GeomagicPosition="0 20 15"
scale3d_skin="0.5 0.65 0.02"
SkinVolume_fileName="C:\sofa\src\VirtualSkinV4\mesh\skin_volume_403020_05" #03 troppo lento
needleVolume_fileName="mesh/straight_needle.obj"

scale3d_needle="5 5 5"
stiffness_springNeedle=100
pointPosition_onNeedle="-6.98 0.02 0.05"

# data
skin_youngModulus=1700#300
thread_youngModulus=2000
skin_poissonRatio=0.49
thread_poissonRatio=0.8


USE_GUI = True


def main():
    import SofaRuntime
    import Sofa.Gui
    import Sofa.Core
    SofaRuntime.importPlugin("SofaComponentAll")
    SofaRuntime.importPlugin("SofaOpenglVisual")
    SofaRuntime.importPlugin("SofaImplicitOdeSolver")

    root = Sofa.Core.Node("root")
    createScene(root)

    Sofa.Simulation.init(root)

    if not USE_GUI:
        for iteration in range(10):
            Sofa.Simulation.animate(root, root.dt.value)
    else:
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.addGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()

def createScene(root):
    [RepNumber,user_name]=read_Files.read()

    fileNamePos=f"Rep{RepNumber}_{user_name}_RingsPos_Double"
    fileNameVel=f"Rep{RepNumber}_{user_name}_RingsVel_Double"
    fileNameForce=f"Rep{RepNumber}_{user_name}_RingsForce_Double"

    # Define root properties
    root.gravity=[0, 0, -2]
    root.dt=0.01

    # Required plugins
    root.addObject('RequiredPlugin', pluginName="Geomagic Sofa.Component.Constraint.Projective Sofa.Component.MechanicalLoad SofaCarving Sofa.Component.Mapping.MappedMatrix Sofa.Component.Constraint.Lagrangian.Model Sofa.Component.Constraint.Lagrangian.Correction Sofa.Component.Constraint.Lagrangian.Solver Sofa.Component.AnimationLoop Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Response.Contact Sofa.Component.SolidMechanics.Spring Sofa.Component.Engine.Select Sofa.Component.IO.Mesh Sofa.Component.Playback Sofa.Component.SolidMechanics.Spring Sofa.Component.Constraint.Projective Sofa.Component.MechanicalLoad Sofa.Component.SolidMechanics.FEM.Elastic Sofa.Component.Haptics Sofa.Component.ODESolver.Backward Sofa.Component.IO.Mesh Sofa.Component.Collision.Geometry Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Response.Mapper Sofa.Component.Collision.Response.Contact Sofa.GL.Component.Rendering2D Sofa.GL.Component.Rendering3D Sofa.GL.Component.Shader Sofa.Component.Mapping Sofa.Component.SolidMechanics.Spring Sofa.Component.Diffusion Sofa.Component.SolidMechanics.FEM.Elastic Sofa.Component.LinearSolver.Direct Sofa.Component.Collision.Geometry Sofa.Component.Collision.Detection.Algorithm Sofa.Component.Collision.Detection.Intersection Sofa.Component.Collision.Response.Contact Sofa.Component.Controller Sofa.GUI.Component Sofa.Component.Topology.Mapping Sofa.Component.Mapping SofaValidation")

    root.addObject("VisualStyle", displayFlags="showVisual hideBehaviorModels")
    root.addObject('ViewerSetting', fullscreen="false")
    root.addObject('BackgroundSetting', color="0.3 0.5 0.8")

    # Collision pipeline
    root.addObject('CollisionPipeline', depth="6", verbose="0", draw="0")

    # Forces
    root.addObject('BruteForceBroadPhase', name="detection")
    root.addObject('BVHNarrowPhase')
    root.addObject('DefaultContactManager', name="CollisionResponse", response="FrictionContactConstraint")
    root.addObject('LocalMinDistance', name="proximity", alarmDistance="0.3", contactDistance="0.05", angleCone="0.0")

    # Animation loop
    root.addObject('FreeMotionAnimationLoop')

    # Constraint solver
    root.addObject('LCPConstraintSolver', tolerance="0.001", maxIt="1000")

    # Create two instances of Skin with different stiffness values
    skin_Up=suture_models.Skin(parentNode=root, 
        name='Dermal', 
        rotation=[0.0, 0.0, 0.0], 
        translation=[0.0, 0.0, 3.0], 
        sphere1Box=[1, 1, -0.1, 1, 1, -0.1], sphere2Box=[1, 1, -0.1, 1, 1, -0.1],
        #fixingBox=[-0.1, -0.1, -2, 10, 20, 0.1],
        scale3d=scale3d_skin)
    skin_Up.stiffness = 80
    print ('Dermal found')
    
    skin_Down=suture_models.Skin(parentNode=root, 
        name='SubDermal', 
        rotation=[0.0, 0.0, 0.0], 
        translation=[0.0, 0.0, 0.0], 
        sphere3Box=[1, 1, -0.1, 1, 1, -0.1], sphere4Box=[1, 1, -0.1, 1, 1, -0.1],
        #fixingBox=[-0.1, -0.1, -2, 10, 20, 0.1], 
        scale3d="0.5 0.65 0.05")
    skin_Down.stiffness = 200
    #skin_Down.SkinVisu.Visual.isEnabled.value = False
    print ('SubDermal found')

    # Add geomagic drivers
    root.addObject('GeomagicDriver', name="GeomagicDeviceRight", deviceName="Default Device", scale="1", drawDeviceFrame="0", 
    drawDevice="0", positionBase="10 10 10",  orientationBase="0.707 0 0 0.707")#, forceFeedBack="@StraightNeedle/LCPFFNeedle")

    # Add geomagic nodes
    GeomagicDevice(parentNode=root, name='OmniRight', position="@GeomagicDeviceRight.positionDevice")
    
    # Add needles
    suture_models.StraightNeedle(parentNode=root, name='StraightNeedle', monitor=True, file1=fileNamePos, file2=fileNameVel, file3=fileNameForce, position="@GeomagicDeviceRight.positionDevice", external_rest_shape='@../OmniRight/DOFs') # To fall on sphere: dx=12, dy=3, dz=6

    # Add Rings controller    
    root.addObject(RingsTaskController(name="MyController", rootNode=root))
    print ('Controller found')

    return root

## This function defines a geomagic
# @param parentNode: parent node of the skin patch
# @param name: name of the behavior node
# @param rotation: rotation 
# @param translation: translation
def GeomagicDevice(parentNode=None, name=None, position=None):
    name=parentNode.addChild(name)
    name.addObject('MechanicalObject', template="Rigid3", name="DOFs", position=position)
    name.addObject('MechanicalStateController', template="Rigid3", listening="true", mainDirection="-1.0 0.0 0.0", handleEventTriggersUpdate="true")

class RingsTaskController(Sofa.Core.Controller):
    def __init__(self, name, rootNode):
        Sofa.Core.Controller.__init__(self, name, rootNode)
        
        self.root=rootNode


if __name__ == '__main__':
    main()




