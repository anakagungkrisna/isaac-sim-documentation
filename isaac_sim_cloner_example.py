from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # enable visualization

from omni.isaac.cloner import Cloner    # import Cloner interface
from omni.isaac.cloner import GridCloner    # import GridCloner interface
from omni.isaac.core.utils.stage import get_current_stage
from pxr import UsdGeom

from omni.isaac.core.world import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.articulations import ArticulationView
from omni.isaac.core.utils.types import ArticulationAction # for single agent
from omni.isaac.core.utils.types import ArticulationActions # for multi agent
import numpy as np
import asyncio

# for unitree a1
from omni.isaac.quadruped.robots import Unitree


# multi tasking as in arduino millis
from time import time
previousTime = int(time() * 1000) # in miliseconds
previousTime2 = int(time() * 1000)
intervalTime1 = 500 # 5 seconds
state = False
print("Time in milliseconds since epoch",previousTime)



class Agents:
    # for more ArticulationView functions to define, visit 
    # https://docs.omniverse.nvidia.com/py/isaacsim/
    # source/extensions/omni.isaac.core/docs/index.html
    def __init__(self, agents_num, robot_name):
        num_of_agents = agents_num
        assets_root_path = get_assets_root_path()
        asset_path = ""
        self.battle_ram = False
        # models which has controller defined does not work?
        if(robot_name == "a1"):
            asset_path = assets_root_path + "/Isaac/Robots/Unitree/a1.usd"
        elif(robot_name == "go1"):
            asset_path = assets_root_path + "/Isaac/Robots/Unitree/go1.usd"
        elif(robot_name == "anymal_instanceable"):    
            asset_path = assets_root_path + "/Isaac/Robots/ANYbotics/anymal_instanceable.usd"
        elif(robot_name == "anymal"):    
            asset_path = assets_root_path + "/Isaac/Robots/ANYbotics/anymal_c.usd"
        elif(robot_name == "carter_v1"):    
            asset_path = assets_root_path + "/Isaac/Robots/Carter/carter_v1.usd"
        elif(robot_name == "carter_v2"):    
            asset_path = assets_root_path + "/Isaac/Robots/Carter/carter_v2.usd"
        elif(robot_name == "aws_robomaker_jetbot"):    
            asset_path = assets_root_path + "/Isaac/Robots/Jetbot/aws_robomaker_jetbot.usd"
        elif(robot_name == "jetbot"):    
            asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        elif(robot_name == "transporter_sensors"):    
            asset_path = assets_root_path + "/Isaac/Robots/Transporter/transporter_sensors.usd"
        elif(robot_name == "kaya"):    
            asset_path = assets_root_path + "/Isaac/Robots/Kaya/kaya.usd"
        elif(robot_name == "o3dyn"):    
            asset_path = assets_root_path + "/Isaac/Robots/O3dyn/o3dyn.usd"
        elif(robot_name == "cobotta_pro_900"):    
            asset_path = assets_root_path + "/Isaac/Robots/Denso/cobotta_pro_900.usd"
        elif(robot_name == "franka"):    
            asset_path = assets_root_path + "/Isaac/Robots/Franka/franka.usd"
        elif(robot_name == "franka_alt_fingers"):    
            asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
        elif(robot_name == "allegro_hand"):    
            asset_path = assets_root_path + "/Isaac/Robots/AllegroHand/allegro_hand.usd"
        elif(robot_name == "shadow_hand"):    
            asset_path = assets_root_path + "/Isaac/Robots/ShadowHand/shadow_hand.usd"
        elif(robot_name == "cf2x"):    
            asset_path = assets_root_path + "/Isaac/Robots/Crazyflie/cf2x.usd"
        elif(robot_name == "quadcopter"):    
            asset_path = assets_root_path + "/Isaac/Robots/Quadcopter/quadcopter.usd"
        else:
            asset_path = "/media/storage/Isaac/soankusu/battle_ram.usd"
            self.battle_ram = True


        #world.scene.add(Unitree(prim_path="/World/A1_0", name="A1_0", position=np.array([0, 0, 0.400])))
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/A1_0")
        print("One agents created!")

        # create a cloner object
        cloner = GridCloner(spacing=1)
        print("Cloner created!")

        # generate num_of_agents paths that begin with "/World/A1" - path will be appended with _{index}
        target_paths = cloner.generate_paths("/World/A1", agents_num)
        # setting the base path for the cloner to clone
        cloner.clone(source_prim_path="/World/A1_0", prim_paths=target_paths)
        print("cloned agents created!")

        #batch process articulation via ArticulationView
        if(self.battle_ram == True):
            self.articulation_view = ArticulationView(prim_paths_expr=
                "/World/A1_.*"+ "/Battle_ram/body", 
                name="battleram_view")
        else:  
            self.articulation_view = ArticulationView(prim_paths_expr=
                "/World/A1_.*", name="A1_view")
                #"/World/A1_[0-" + str(num_of_agents-1)+ "]", name="A1_view")
        print("articulationview for the agents defined!")

        self.num_dof = self.articulation_view.num_dof
        self.prim_paths = self.articulation_view.prim_paths

    def set_world_poses(self, positions):
        print("setting world poses")
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
num_agents = 15
print("creating agents")
agents = Agents(num_agents, "a1")
world.scene.add_default_ground_plane()

# View classes are internally initialized when they are added to the scene and the world is reset
world.scene.add(agents.articulation_view)
print("resetting world")
world.reset()
print("finish resetting world")


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
            actions.joint_efforts = np.random.randint(-50,50,(num_agents,agents.articulation_view.num_dof))
            agents.send_actions(actions)
            state = False
        else:
            actions= ArticulationActions()
            actions.joint_efforts = np.random.randint(-50,50,(num_agents,agents.articulation_view.num_dof))
            agents.send_actions(actions)
            state = True
        print(state)
        print("for all agents: ")
        agents.get_actions()
        agents.get_joints_state()
        agents.get_velocities()
        agents.get_local_poses()
        agents.get_world_poses()
        #print(agents.articulation_view.num_dof)
        #print(agents.articulation_view.prim_paths)

    # execute one physics step and one rendering step
    world.step(render=True) 
    current_episode += 1

# Dont forget to properly close the simulation
simulation_app.close() # close Isaac Sim