#!/usr/bin/env python3

import rospy
from rospy.exceptions import ROSInterruptException
from std_msgs.msg import Bool
from eezy_arm_ros.msg import Joints, JointsArray
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import numpy as np
from scipy.interpolate import CubicSpline
from time import sleep, time

class MotorController:
    def __init__(self):
        # These are dependent on how our arm was built
        # Can set for your arm in the roslaunch file
        self.ports = rospy.get_param("ports", [2, 4, 17, 22])
        self.offsets = rospy.get_param("jt_offsets", [0, 5, -8])
        self.ee_angle_open = rospy.get_param("ee_open", 0.0)
        self.ee_angle_closed = rospy.get_param("ee_closed", -90.0)
        self.curr_q = np.zeros(3)
        
        # These should be universal
        # Scale used to take into account gearing ratio, and servo 
        # mount orientation
        self.scale = [2, 1, -1]
        # Limits are based on the "scaled" angles
        self.lower_limits = [-45, -40, -25]
        self.upper_limits = [ 45,  40,  50]
        self.dt = 1.0/60

        # setup ros subscribers/publishers
        self.q_pub = rospy.Publisher("q", Joints, queue_size=10)
        self.ee_sub = rospy.Subscriber("ee_closed", Bool, self.listen_ee)
        self.q_des_sub = rospy.Subscriber("q_des", Joints, self.listen_jts)
        self.q_des_traj_sub = rospy.Subscriber("q_des_traj", JointsArray, self.listen_jts_traj)

        # These are the parameters for the MG996R
        factory = PiGPIOFactory()
        self.servos = [Servo(p, min_pulse_width=0.5/1000,
                            max_pulse_width=2.5/1000,
                            initial_value=None,
                            pin_factory=factory) for p in self.ports]
        # move to zero config
        self.command_servos(self.curr_q)

    def tearDown(self):
        """Stops the servos when exiting"""
        for servo in self.servos:
            servo.detach()

    ##################################################
    #             HANDLE SERVO CONTROL               #
    ##################################################
    def command_servos(self, val):
        """Process joint angles, sends to servo, and publishes current q"""
        self.curr_q = val
        for i in range(3):
            clipped = max(self.lower_limits[i], min(val[i], self.upper_limits[i]))
            self.servos[i].value = (clipped*self.scale[i] + self.offsets[i])/90
        
        # Send commanded q
        self.q_pub.publish(Joints(*self.curr_q))


    def command_traj(self, qs):
        """Sends a given trajectory to the joints"""
        max_a = rospy.get_param("max_accel", 10000)

        # figure how long to give to each point & joint
        tf = np.sqrt(np.abs(np.diff(qs, axis=0))*6.0 / max_a)
        # take the longest joint for each point
        tf = tf.max(axis=1)
        t = np.append(0, np.cumsum(tf))

        # interpolate
        c = CubicSpline(t, qs, bc_type='clamped')

        # send to servo
        for ti in np.arange(0, t[-1], self.dt):
            start = time()
            self.command_servos(c(ti))
            # sleep dt minus whatever it took to command the motors
            diff = time() - start
            if diff < self.dt:
                sleep(self.dt - diff)

    ##################################################
    #              CALLBACK FUNCTIONS                #
    ##################################################
    def listen_jts_traj(self, data):
        """Callback that listens for a desired joint position."""
        qs = np.array([[q.q1, q.q2, q.q3] for q in data.data])
        # add in current q
        qs = np.vstack((self.curr_q, qs))
        rospy.loginfo(f"Moving jts through trajectory")
        self.command_traj(qs)

    def listen_jts(self, data):
        """Callback that listens for a desired joint position."""
        # add in current q
        qs = np.array([self.curr_q, [data.q1, data.q2, data.q3]])
        rospy.loginfo(f"Moving jts to {qs[-1]}")
        self.command_traj(qs)
        
    # Let closed = true
    def listen_ee(self, data):
        """Callback that listens for opening/closing the claw."""
        des_closed = data.data

        if des_closed:
            rospy.loginfo("Closing claw")
            self.servos[-1].value = self.ee_angle_closed / 90.0
        else:
            rospy.loginfo(f"Opening claw")
            self.servos[-1].value = self.ee_angle_open / 90.0



if __name__ == "__main__":
    try:
        rospy.init_node("controller")
        motor_controller = MotorController()
        rospy.spin()
    except ROSInterruptException:
        motor_controller.tearDown()
