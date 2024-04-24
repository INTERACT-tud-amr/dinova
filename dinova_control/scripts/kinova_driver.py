#!/usr/bin/env python3

import rospy
from threading import Thread
import time
import sys
import numpy as np

from kinova_ros_interface.kinova_client import KinovaRobot
from kinova_ros_interface.utilities import DeviceConnection
from kinova_ros_interface.state import State

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64, Bool, Int32, Empty
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Joy

PUBLISH_RATE = 40 #Hz


class ControlInterface():
    """A node that starts the control interface of the robot."""
    def __init__(self, mode="position") -> None:
        self.mode = mode

        self.pub_feedback = rospy.Publisher('/kinova/joint_states', JointState, queue_size=10)
        rospy.Subscriber("/kinova/command", Float64MultiArray, self.callback_command)
        rospy.Subscriber("/kinova/gripper", Float64, self.callback_gripper)
        rospy.Subscriber("/kinova/error_ack", Empty, self.callback_error_ack)

        # Emergency switch
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.callback_emergency_switch)
        self.emergency_switch_pressed = False

        # Services for setting predefined joint positions
        rospy.Service("/kinova/go_home_position", Trigger, self.handle_go_home_pos)
        rospy.Service("/kinova/go_zero_position", Trigger, self.handle_go_zero_pos)
        # Services for changing control modes
        rospy.Service("/kinova/change_to_LLC_position", Trigger, self.handle_LLC_position)
        rospy.Service("/kinova/change_to_LLC_velocity", Trigger, self.handle_LLC_velocity)
        rospy.Service("/kinova/change_to_HLC_position", Trigger, self.handle_HLC_position)
        rospy.Service("/kinova/change_to_HLC_velocity", Trigger, self.handle_HLC_velocity)


        self.state = State()
        self.start_threads()

        """Start the robot """
        with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
            alpha_dq = rospy.get_param("/kinova_feedback/alpha_dq")
            alpha_torque = rospy.get_param("/kinova_feedback/alpha_torque")
            self.kinova = KinovaRobot(router=router, 
                                      real_time_router=real_time_router, 
                                      state = self.state, 
                                      alpha_dq= alpha_dq,
                                      alpha_torque = alpha_torque)
            print("Current control mode:", self.mode)
            self.kinova.start_feedback()



    def start_threads(self):
            self.publishing_active = True
            publish_thread = Thread(target=self.start_publish_loop)
            publish_thread.start()

            spin_thread = Thread(target=self.start_spin_loop)
            spin_thread.start()

    
    def start_publish_loop(self):
        while self.publishing_active:
            self.publish_feedback()
            time.sleep(1/ PUBLISH_RATE)
    
    def start_spin_loop(self):
        rospy.spin()
        if rospy.is_shutdown():
            self.publishing_active = False
            self.kinova.stop_feedback()
            self.kinova.base.Stop()
            sys.exit()
    
    def publish_feedback(self):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        # for i in range(self.kinova.actuator_count):
        for i in range(len(self.state.kinova_feedback.q)):
            name = "joint_" + str(i+1)
            js_msg.name.append(name)
            js_msg.position.append(self.state.kinova_feedback.q[i])
            js_msg.velocity.append(self.state.kinova_feedback.dq[i])
            js_msg.effort.append(self.state.kinova_feedback.torque[i])
        self.pub_feedback.publish(js_msg)

    def stop_threads(self):
        self.publishing_active = False
        self.kinova.stop_feedback()
        

    def callback_command(self, msg):
        if not self.emergency_switch_pressed:
            if self.mode == "HLC_position":
                self.state.kinova_command.q = np.rad2deg(msg.data)
                self.kinova.set_high_level_position(self.state.kinova_command.q)
            elif self.mode == "HLC_velocity":
                self.state.kinova_command.dq = np.rad2deg(msg.data)
                self.kinova.set_high_level_velocity(self.state.kinova_command.dq)
            elif self.mode == "LLC_velocity":   #TODO: not working
                self.kinova.joints_command = np.rad2deg(msg.data)
                # print("Not yet implemented. Command ignored.")

            elif self.mode == "LLC_position":
                print("Not yet implemented. Command ignored.")
    
    def callback_emergency_switch(self, msg):
        if msg.buttons[4]:
            self.emergency_switch_pressed = True
            self.set_zero_velocity()
        else:
            self.emergency_switch_pressed = False


    def callback_gripper(self, msg):
        self.kinova._move_gripper(msg.data)

    def callback_error_ack(self, msg):
        self.kinova.clear_faults()

    
    def handle_go_home_pos(self, req):
        self.state.kinova_command.q = [0, -16, 75, 0, -60, 0]
        success = self.kinova.set_high_level_position(self.state.kinova_command.q)
        return success, ""
    
    def handle_go_zero_pos(self, req):
        self.state.kinova_command.q = self.kinova.actuator_count * [0.]
        success = self.kinova.set_high_level_position(self.state.kinova_command.q)
        return success, ""

    def handle_HLC_position(self, req):
        self.mode = "HLC_position"
        self.kinova.disconnect_LLC()
        success, _ = self.kinova.stop_LLC()
        print("Control Mode changed to: ", self.mode)
        return success, self.mode
    
    def handle_HLC_velocity(self, req):
        self.mode = "HLC_velocity"
        self.kinova.disconnect_LLC()
        success, _ = self.kinova.stop_LLC()
        print("Control Mode changed to: ", self.mode)
        return success, self.mode

    def handle_LLC_position(self, req):
        self.mode = "LLC_position"
        self.kinova.start_LLC()
        print("Control Mode changed to: ", self.mode)
        return True, self.mode
    
    def handle_LLC_velocity(self, req):
        self.mode = "LLC_velocity"
        self.kinova.start_LLC()
        self.kinova.connect_LLC()
        print("Control Mode changed to: ", self.mode)
        return True, self.mode

    def set_zero_velocity(self):
        self.state.kinova_command.dq = self.kinova.actuator_count * [0.]
        self.kinova.set_high_level_velocity(self.state.kinova_command.dq)


        


if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("kinova_driver")

    kinova_driver = ControlInterface("HLC_velocity")
