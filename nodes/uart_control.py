#!/usr/bin/env python3

# System imports
import serial

# ROS imports
import rospy
from rospy.exceptions import ROSInterruptException
from std_msgs.msg import String

# mini UART on Raspberry Pi
com_port = rospy.get_param("com_port", "/dev/ttyS0")
baud_rate = rospy.get_param("baud_rate", 9600)
timeout_seconds = 1.0
ser = serial.Serial(com_port, baud_rate, timeout=timeout_seconds)


def talker():
    while not rospy.is_shutdown():
        data = ser.read(20)  # FIXME, how many characters?
        data_str = data.decode('utf-8')
        # rospy.loginfo(data_str)
        pub.publish(String(data_str))
        ser.write(data)  # Temp, for verification


if __name__ == "__main__":
    try:
        pub = rospy.Publisher('/uart_commands', String, queue_size=10)
        rospy.init_node('uart_control')
        talker()
    except ROSInterruptException:
        ser.close()