#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# This probably needs to be more sophisticated eventually
def listen_jts(data):
    val = [data.x, data.y, data.z]
    rospy.loginfo(f"Setting jts to {val}")
    for i in range(3):
        servo[i].value = (val[i]*scale[i] + offsets[i])/90
    
# Let closed = true
def listen_ee(data):
    des_closed = data.data

    if des_closed:
        rospy.loginfo("Closing claw")
        servo[-1].value = ee_closed / 90.0
    else:
        rospy.loginfo(f"Opening claw")
        servo[-1].value = ee_open / 90.0

if __name__ == "__main__":
    # These are dependent on how our arm was built
    # Can set for your arm in the roslaunch file
    ports = rospy.get_param("ports", [2, 4, 17, 22])
    offsets = rospy.get_param("jt_offsets", [0, 5, -8])
    ee_open = rospy.get_param("ee_open", 0.0)
    ee_closed = rospy.get_param("ee_closed", -90.0)
    
    # This should be universal
    scale = [2, 1, -1]

    # These are the parameters for the MG996R
    factory = PiGPIOFactory()
    servo = [Servo(p, min_pulse_width=0.5/1000,
                        max_pulse_width=2.5/1000,
                        initial_value=None,
                        pin_factory=factory) for p in ports]

    rospy.init_node("controller")
    rospy.Subscriber("q_des", Vector3, listen_jts)
    rospy.Subscriber("ee_des", Bool, listen_ee)

    rospy.spin()