#!/usr/bin/env python3
import numpy as np

# ROS imports
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray

NUM_JOINTS = 3

class EezyBotArm:
    def __init__(self) -> None:
        # Number of joints (base, vertical arm, horizontal arm)
        self.n = NUM_JOINTS
        # Current joint angles (servos 1, 2, 3)
        self.q_des = [None, None, None]
        self.q = [None, None, None]
        # Keep track of the end effector status (open/closed)
        self.ee_closed = True

        # Set up ROS publishers and subscribers
        self.fk_pub = rospy.Publisher("/forward_kinematics", Float32MultiArray, queue_size=10)
        self.q_subscriber = rospy.Subscriber("/q_des", Vector3, self.fk_callback)
   
    def fk_callback(self, msg : Vector3):
        """ROS callback for computing forward kinematics. Subscribes to the desired joint configuration.

        Args:
            msg (Vector3): ROS message containing the current joint angles
        """
        self.q_des = [msg.x, msg.y, msg.z]
        self.forward_kinematics()

    def forward_kinematics(self, starting_joint_idx=0, ending_frame_idx=NUM_JOINTS, base=True, ee=True) -> np.array:
        """Returns the transformation matrix from one link to another given the current joint configuration (self.q).
           Defaults to calculating fk from the base to the end effector.

        Args:
            starting_joint_idx (int, optional): Index of the starting joint. Defaults to 0 (base frame).
            ending_frame_idx (int, optional): Index of the ending frame. Defaults to NUM_JOINTS (end effector frame).
            base (bool, optional): If true and starting index is 0, then base transform will be included. Defaults to True.
            ee (bool, optional): If true and index ends at nth frame, then end effector transforma will be included. Defaults to True.

        Returns:
            np.array: 4x4 transformation matrix from the 'starting_joint' to the 'ending_frame'
        """
        T = np.eye(4)
        return T

if __name__ == "__main__":
    rospy.init_node("kinematics")
    EezyBotArm()
    rospy.spin()
