import rospy
import numpy as np
from eezy_arm_ros.msg import Vector3Array
from geometry_msgs.msg import Vector3
from time import sleep
import sys

rospy.init_node("trajectory_creator")
pub = rospy.Publisher("/arm1/p_des_traj", Vector3Array, queue_size=10)
while pub.get_num_connections() == 0:
    print("Waiting for connections...")
    sleep(1)

t = np.linspace(-np.pi/2, 3*np.pi/2, 30)

traj = np.array([200+0*t, 150*np.cos(t), 225+75*np.sin(t)*np.cos(t)]).T

msg = Vector3Array([Vector3(*p) for p in traj])
pub.publish(msg)

print("Figure 8 sent")