import rospy
from std_msgs.msg import Bool
from time import sleep
import sys

rospy.init_node("move_claw")
pub = rospy.Publisher("/arm1/ee_closed", Bool, queue_size=10)
while pub.get_num_connections() == 0:
    print("Waiting for connections...")
    sleep(1)

# make our line
if len(sys.argv) < 2:
    print("You need to input an option")

if sys.argv[1] in ["1", "True", "true", "open", "y", "yes"]:
    msg = True
else:
    msg = False

pub.publish(msg)
print("Sent message.")
