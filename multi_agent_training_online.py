# This standalone application is based on: 
# https://docs.omniverse.nvidia.com/py/isaacsim/
# source/extensions/omni.isaac.core/docs/
# https://docs.omniverse.nvidia.com/app_isaacsim/
# app_isaacsim/tutorial_core_hello_world.html
# https://docs.omniverse.nvidia.com/app_isaacsim/
# app_isaacsim/tutorial_gym_new_rl_example.html

#launch Isaac Sim before any other imports
#must include these default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # enable visualization

import omni
from pxr import Gf, Sdf, UsdPhysics
from pxr import PhysxSchema
from pxr import PhysicsSchemaTools
import numpy as np
import time

# libraries used for deplying multiple agents
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.world import World
import asyncio
from omni.isaac.core.utils.types import ArticulationAction # for single agent
from omni.isaac.core.utils.types import ArticulationActions # for multi agent

# multi tasking as in arduino millis
from time import time
previousTime = int(time() * 1000) # in miliseconds
intervalTime1 = 100 # 5 seconds
state = False
print("Time in milliseconds since epoch",previousTime)

class Agents:
    # for more ArticulationView functions to define, visit 
    # https://docs.omniverse.nvidia.com/py/isaacsim/
    # source/extensions/omni.isaac.core/docs/index.html
    def __init__(self, agents_num):
        #add battle ram articulation (use complete path)
        self.usd_path = "/media/storage/Isaac/soankusu/battle_ram.usd"
        num_of_agents = agents_num
        for i in range(num_of_agents):
            add_reference_to_stage(usd_path=self.usd_path, 
                prim_path="/World/battleram_" + str(i+1))
        print("Agents created!")

        #batch process articulation via ArticulationView
        self.articulation_view = ArticulationView(prim_paths_expr=
            "/World/battleram_.*"+ "/Battle_ram/body", 
            name="battleram_view")
        print("articulationview for the agents defined!")

    def set_world_poses(self, positions):
        self.articulation_view.set_world_poses(positions)
        #print("world poses applied")

    def send_actions(self, actions):
        self.articulation_view.apply_action(actions)
    
    def get_actions(self):
        applied_actions = self.articulation_view.get_applied_actions()
        print("applied actions are: ", applied_actions.joint_velocities)

    def get_jacobians(self):
        jacobian_matrices = self.articulation_view.get_jacobians()
        print(jacobian_matrices)

    def get_joints_state(self):
        # current joint positions and velocities.
        joints_state = self.articulation_view.get_joints_state()
        positions = joints_state.positions
        velocities = joints_state.velocities
        efforts = joints_state.efforts
        print("positions: ", positions)
        print("velocities: ", velocities)
        print("efforts: ", efforts)

    def get_velocities(self):
        # linear and angular velocities of the prims in the view concatenated. 
        # shape is (M, 6).
        velocities = self.articulation_view.get_velocities()
        print("linear & angular velocities: ", velocities)

    def get_local_poses(self):
        # Gets prim poses in the view 
        # with respect to the local frame (the prim’s parent frame).
        local_poses = self.articulation_view.get_local_poses()
        print("local poses: ", local_poses)

    def get_world_poses(self):
        # Gets the poses of the prims in the view 
        # with respect to the world’s frame.
        world_poses = self.articulation_view.get_world_poses()
        print("world poses: ", world_poses)




world = World()
world.scene.add_default_ground_plane()

print("creating agents")
num_agents = 50
agents = Agents(num_agents)
world.scene.add(agents.articulation_view)

world.reset()

#set root body pose of the agents based on world frame
#new_positions = np.array([[0.0, 50.0, 0.0], 
#    [50.0, 10.0, 0.0]]) # for 2 agent
#agents.set_world_poses(new_positions)

#the main loop for training
current_episode = 0
total_episodes = 5000
#while current_episode < total_episodes:
while True:
    currentTime = int(time() * 1000)
    if(currentTime - previousTime > intervalTime1):
        previousTime = currentTime
        print("passing 2.5 seconds")
        if(state == True):
            #create articulation actionS to send command to all articulation root
            actions= ArticulationActions()
            actions.joint_velocities = np.random.randint(-50,50,(num_agents,agents.articulation_view.num_dof))
            agents.send_actions(actions)
            state = False
        else:
            actions= ArticulationActions()
            actions.joint_velocities = np.random.randint(-50,50,(num_agents,agents.articulation_view.num_dof))
            agents.send_actions(actions)
            state = True
        print(state)
        print("for all agents: ")
        agents.get_actions()
        agents.get_joints_state()
        agents.get_velocities()
        agents.get_local_poses()
        agents.get_world_poses()

    #world.pause()

    # we have control over stepping physics and rendering in this workflow
    # things run in sync
    world.step(render=True) # execute one physics step and one rendering step
    current_episode += 1

simulation_app.close() # close Isaac Sim