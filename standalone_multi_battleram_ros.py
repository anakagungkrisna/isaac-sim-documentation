#launch Isaac Sim before any other imports
#must include these default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # enable visualization


import omni
from pxr import Gf, Sdf, UsdPhysics
from pxr import PhysxSchema
from pxr import PhysicsSchemaTools
import numpy as np

# libraries used for deploying multiple agents with ros nodes and creating the world
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.world import World
from omni.isaac.core.utils.extensions import enable_extension
import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path


# enable ROS bridge extension
enable_extension("omni.isaac.ros_bridge")
simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph
if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()



if World.instance():
    print("World exist")
    World.instance().clear_instance()
world = World(stage_units_in_meters=1.0)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

#using warehouse environments 
prim = get_prim_at_path("/World/Warehouse")
if not prim.IsValid():
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

"""
# defining your own world 
stage = omni.usd.get_context().get_stage()
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(98.10)

PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/physicsScene"))
physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/physicsScene")
physxSceneAPI.CreateEnableCCDAttr(True)
physxSceneAPI.CreateEnableStabilizationAttr(True)
physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
physxSceneAPI.CreateSolverTypeAttr("TGS")

stage = omni.usd.get_context().get_stage()
PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 100, Gf.Vec3f(0, 0, -100), Gf.Vec3f(1.0))
"""

#add battle ram articulation
usd_path = "/media/storage/Isaac/soankusu/battle_ram.usd"
num_of_agents = 3
for i in range(num_of_agents):
    add_reference_to_stage(usd_path=usd_path, prim_path="/World/battleram_" + str(i+1))

world.reset()
print("finished spawning robots")

for i in range(num_of_agents):
    og.Controller.edit(
        {
            "graph_path": "/World/battleramnode_" + str(i+1), 
            "evaluator_name": "execution",
            #"pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND, #dont use this
        },
        
        {
            og.Controller.Keys.CREATE_NODES: [
                # (name, node:type) -> you can search for node:type in the raw USH properties of the prim
                #("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("OnTick", "omni.graph.action.OnTick"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("publishClock", "omni.isaac.ros_bridge.ROS1PublishClock"),
                
                ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
                ("SubscribeJointState", "omni.isaac.ros_bridge.ROS1SubscribeJointState"),
            ],
            og.Controller.Keys.CONNECT: [
                #("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                #("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                #("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnTick.outputs:tick", "ArticulationController.inputs:execIn"),
                
                ("OnTick.outputs:tick", "publishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),

                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:usePath", True),
                ("ArticulationController.inputs:robotPath", "/World/battleram_" + str(i+1) + "/Battle_ram/body"),
                ("PublishJointState.inputs:topicName", "joint_states_battleram_" + str(i+1)),
                ("SubscribeJointState.inputs:topicName", "joint_command_battleram"),
            ],
        },
    )

    #setting the robot target prim to publish JointState node
    #cant you set this directly to the node??? no... this method is used so that we can use the  "Add Target(s) " button in the property panel
    set_target_prims(primPath="/World/battleramnode_"+ str(i+1)+"/PublishJointState", targetPrimPaths=["/World/battleram_" + str(i+1) + "/Battle_ram/body"])
    
    print("created node")
    
print("finished creating ros utilities")



if __name__ == "__main__":

    while simulation_app.is_running():
        world.step(render=True)
    
    rospy.signal_shutdown("completed the standalone battleram program")
    simulation_app.close()