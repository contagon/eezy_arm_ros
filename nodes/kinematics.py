#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as scipy_R

# ROS imports
import rospy
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion

NUM_JOINTS = 3


def deg2rad(deg : float) -> float:
    """Helper function converts degrees to radians

    Args:
        deg (float): Angle in degrees

    Returns:
        float: Angle in radians
    """
    return deg * np.pi / 180.


def rotate_x_se3(theta : float, radians=False) -> np.array:
    """Computes a transformation matrix of a rotation of theta about the x axis

    Args:
        theta (float): Angle, in degrees or radians
        radians (bool, optional): Determines if angle is in degrees or radians. Defaults to False (degrees).
    
    Returns:
        np.array: 4x4 transformation matrix (rotation only)
    """
    if not radians:
        theta = deg2rad(theta)

    return np.array([[1, 0,              0,             0],
                     [0, np.cos(theta), -np.sin(theta), 0],
                     [0, np.sin(theta),  np.cos(theta), 0],
                     [0, 0,              0,             1]])


def rotate_y_se3(theta : float, radians=False) -> np.array:
    """Computes a transformation matrix of a rotation of theta about the y axis

    Args:
        theta (float): Angle, in degrees or radians
        radians (bool, optional): Determines if angle is in degrees or radians. Defaults to False (degrees).
    
    Returns:
        np.array: 4x4 transformation matrix (rotation only)
    """
    if not radians:
        theta = deg2rad(theta)

    return np.array([[ np.cos(theta), 0, np.sin(theta), 0],
                     [ 0,             1, 0,             0],
                     [-np.sin(theta), 0, np.cos(theta), 0],
                     [0,              0, 0,             1]])


def rotate_z_se3(theta : float, radians=False) -> np.array:
    """Computes a transformation matrix of a rotation of theta about the z axis

    Args:
        theta (float): Angle, in degrees or radians
        radians (bool, optional): Determines if angle is in degrees or radians. Defaults to False (degrees).
    
    Returns:
        np.array: 4x4 transformation matrix (rotation only)
    """
    if not radians:
        theta = deg2rad(theta)

    return np.array([[np.cos(theta), -np.sin(theta), 0, 0],
                     [np.sin(theta),  np.cos(theta), 0, 0],
                     [0,              0,             1, 0],
                     [0,              0,             0, 1]])


def translate_se3(t_row_vector : np.array) -> np.array:
    """Creates a transformation matrix that is strictly a translation. np.concatenate raises an error 
    if t_row_vector is not the correct dimension.

    Args:
        t_row_vector (np.array): Translation vector (1x3 row vector)

    Returns:
        np.array: 4x4 transformation matrix (translation only)
    """   
    R = np.eye(3)
    # "The -1 on reshape is a broadcast-like, using all possible elements, and the 1 creates the 
    # second required dimension." -- https://stackoverflow.com/a/53304024
    T = np.concatenate((R, t_row_vector.reshape(-1,1)), axis=1)
    T = np.concatenate((T, np.array([0, 0, 0, 1])), axis=0)
    return T


class EezyBotArm:
    def __init__(self) -> None:
        # Number of joints (base, vertical arm, horizontal arm)
        self.n = NUM_JOINTS
        # Keep track of the end effector status (open/closed)
        self.ee_closed = True

        # Robot physical parameters
        self._l1 = 92  # mm from frame 0 to frame 1
        self._l2 = 135  # mm from frame 1 to frame 2
        self._l3 = 147  # mm from frame 2 to frame 3
        self._l4 = 87  # mm from frame 3 to frame 4 

        # Set up ROS publishers and subscribers
        self.fk_publisher = rospy.Publisher("/end_effector_pose", Pose, queue_size=10)
        self.q_subscriber = rospy.Subscriber("/q_des", Vector3, self.fk_callback)

    def compute_A_matrix(self, q : list, joint_idx : int) -> np.array:
        """Computes the transformation matrix from the given joint to the end of the link. With the 
        EEZYbotARM MK2 it does not necessarily follow DH convention. The horizontal link and the end effector 
        are at a fixed orientation (level to the ground) so we have to negate the rotation angle of joint for 
        joints 1 and 2

        Args:
            q (list): Joint angles in degrees
            joint_idx (int): Joint index of the robot

        Returns:
            np.array: 4x4 transformation matrix
        """
        # TODO: Check the angles for radians or degrees
        if joint_idx == 0:
            A = rotate_z_se3(q[0]) @ translate_se3(np.array([0, 0, self._l1])) @ rotate_x_se3(-np.pi/2, radians=True)
        elif joint_idx == 1:
            A = rotate_z_se3(q[1]) @ rotate_z_se3(-np.pi/2, radians=True) @ translate_se3(np.array([self._l2, 0, 0])) @ rotate_z_se3(-np.pi/2, radians=True) @ rotate_z_se3(-q[1])
        elif joint_idx == 2:
            A = rotate_z_se3(q[2]) @ translate_se3(np.array([self._l3, 0, 0])) @ rotate_z_se3(-q[2])
        elif joint_idx == 3:
            A = translate_se3(np.array([self._l4, 0, 0])) @ rotate_x_se3(np.pi/2, radians=True)
        else:
            raise ValueError('Invalid joint! Expects 0 - 3')

        return A
    
    def fk_callback(self, msg : Vector3):
        """ROS callback for computing forward kinematics. Subscribes to the desired joint configuration.

        Args:
            msg (Vector3): ROS message containing the current joint angles in degrees
        """
        q = [msg.x, msg.y, msg.z]
        T_ee = self.forward_kinematics(q)
        
        t = Point()
        t.x = T_ee[0,3]
        t.y = T_ee[1,3]
        t.z = T_ee[2,3]

        quat = Quaternion()
        r = scipy_R.from_matrix(T_ee[0:3, 0:3])
        r = r.as_quat()
        quat.x = r[0][0]
        quat.y = r[0][1]
        quat.z = r[0][2]
        quat.w = r[0][3]
        
        msg_out = Pose()
        msg_out.position = t
        msg_out.orientation = quat

        self.fk_publisher.publish(msg_out)

    def forward_kinematics(self, q : list, starting_link=0, ending_link=NUM_JOINTS, base=True, ee=True) -> np.array:
        """Returns the transformation matrix from one link to another given the desired joint configuration (self.q_des).
           Defaults to calculating fk from the base to the end effector.

        Args:
            q (list): Joint angles in degrees
            starting_link (int, optional): The link from which to calculate fk. Defaults to 0 (base frame).
            ending_frame (int, optional): The link to which we want fk calcualted. Defaults to NUM_JOINTS (end effector frame).
            base (bool, optional): If true and starting index is 0, then base transform will be included. Defaults to True.
            ee (bool, optional): If true and index ends at nth frame, then end effector transforma will be included. Defaults to True.

        Returns:
            np.array: 4x4 transformation matrix from the 'starting_joint' to the 'ending_frame'
        """
        T = np.eye(4) 

        for i in range(starting_link, ending_link):
            T = T @ self.compute_A_matrix(q, i)

        if ee == True and ending_link == NUM_JOINTS:
            T = T @ self.compute_A_matrix(q, 3)

        return T

if __name__ == "__main__":
    rospy.init_node("kinematics")
    EezyBotArm()
    rospy.spin()
