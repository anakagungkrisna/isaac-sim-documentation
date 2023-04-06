import omni.graph.core as og

og.Controller.edit(
    #graph_path -> the path and name for your graph
    {"graph_path": "/World/simple_controller", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            # (name, node:type) 
            # name -> any name for your node
            # node:type -> you can search for it in the raw USD properties of the prim property
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),

            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
            ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController"),
            
            ("MakeArray", "omni.graph.nodes.MakeArray"),
            ("ConstantToken_left", "omni.graph.nodes.ConstantToken"),
            ("ConstantToken_right", "omni.graph.nodes.ConstantToken"),
        ],
        og.Controller.Keys.CONNECT: [
            # (name.tab:key, name.tab:key# (name, node:type) -> you can search for node:type in the raw USH properties of the prim) 
            # tab -> elements of the specific prim property
            # key -> elements of the specific tab
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
            
            ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("MakeArray.outputs:array", "ArticulationController.inputs:jointNames"),
            
            ("ConstantToken_left.inputs:value", "MakeArray.inputs:a"),
            ("ConstantToken_right.inputs:value", "MakeArray.inputs:b"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # (name.tab:key, value) 
            # value -> elements of the specific prim property
            ("ArticulationController.inputs:usePath", True),
            ("ArticulationController.inputs:robotPath", "/World/battle_ram/Battle_ram/body"),
            ("DifferentialController.inputs:wheelDistance", 75.0),
            ("DifferentialController.inputs:wheelRadius", 50.0),
            ("MakeArray.inputs:arraySize", 2),
            ("ConstantToken_left.inputs:value", "front_left_wheel_joint"),
            ("ConstantToken_right.inputs:value", "front_right_wheel_joint"),

            ("DifferentialController.inputs:linearVelocity", -500.0),
            ("DifferentialController.inputs:angularVelocity", 100.0),
        ],
    },
)
