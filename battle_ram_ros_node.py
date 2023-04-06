import omni.graph.core as og
from omni.isaac.core_nodes.scripts.utils import set_target_prims


og.Controller.edit(
    {"graph_path": "/World/BackWheelPubSub", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            # (name, node:type) -> you can search for node:type in the raw USH properties of the prim
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PublishJointState", "omni.isaac.ros_bridge.ROS1PublishJointState"),
            ("SubscribeJointState", "omni.isaac.ros_bridge.ROS1SubscribeJointState"),
            ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
            
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),

            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),

            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ArticulationController.inputs:usePath", True),
            ("ArticulationController.inputs:robotPath", "/World/battle_ram/Battle_ram/body"),
            ("PublishJointState.inputs:topicName", "joint_states_battleram"),
            ("SubscribeJointState.inputs:topicName", "joint_command_battleram"),
        ],
    },
)

#setting the robot target prim to publish JointState node
#cant you set this directly to the node??? no... this method is used so that we can use the  "Add Target(s) " button in the property panel
set_target_prims(primPath="/World/BackWheelPubSub/PublishJointState", targetPrimPaths=["/World/battle_ram/Battle_ram/body"])