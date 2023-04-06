import rospy
from sensor_msgs.msg import JointState
import numpy as np

rospy.init_node("battle_ram_publisher")
pub = rospy.Publisher("/joint_command_battleram", JointState, queue_size=10)

joint_state = JointState()

joint_state.name = ["back_right_wheel_joint", "front_right_wheel_joint", "front_left_wheel_joint", "back_left_wheel_joint", "cone_joint"]


rate = rospy.Rate(10)
while not rospy.is_shutdown():
    joint_state.velocity = np.random.uniform(-50.0,50.0,5)
    pub.publish(joint_state)
    rate.sleep()