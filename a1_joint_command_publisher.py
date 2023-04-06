import rospy
from sensor_msgs.msg import JointState
import numpy as np

rospy.init_node("a1_joint_command_publisher")
pub = rospy.Publisher("/isaac_a1/joint_torque_cmd", JointState, queue_size=10)

joint_state = JointState()

joint_state.name = [
            "FL0",
            "FL1",
            "FL2",
            "FR0",
            "FR1",
            "FR2",
            "RL0",
            "RL1",
            "RL2",
            "RR0",
            "RR1",
            "RR2",
            "FL_foot",
            "FR_foot",
            "RL_foot",
            "RR_foot",
        ]

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    joint_state.effort = np.random.uniform(-50.0,50.0,16)

    pub.publish(joint_state)
    rate.sleep()