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

# make our line
if len(sys.argv) < 2:
    print("You need to input an option")
if sys.argv[1] == 'x':
    traj = np.linspace([100, 0, 250], [300, 0, 250], 20)
    print("Sending x trajectory")
elif sys.argv[1] == 'y':
    traj = np.linspace([200, -200, 250], [200, 200, 250], 20)
    print("Sending y trajectory")
elif sys.argv[1] == 'z':
    traj = np.linspace([200, 0, 150], [200, 0, 250], 20)
    print("Sending z trajectory")
else:
    print("Bad option")


msg = Vector3Array([Vector3(*p) for p in traj])
pub.publish(msg)