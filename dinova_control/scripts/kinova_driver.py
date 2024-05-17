#!/usr/bin/env python3

import rospy
from threading import Thread
import time
import sys
import numpy as np

from dinova_control.kinova_client import KinovaRobot
from dinova_control.utilities import DeviceConnection
from dinova_control.state import State

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64, Bool, Int32, Empty
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Joy
from dingo_msgs.msg import Lights

PUBLISH_RATE = 40 #Hz


class ControlInterface():
    """A node that starts the control interface of the robot."""
    def __init__(self, mode="HLC_velocity") -> None:
        self.mode = mode

        self.pub_feedback = rospy.Publisher('/kinova/joint_states', JointState, queue_size=10)
        rospy.Subscriber("/kinova/command", Float64MultiArray, self.callback_command)
        rospy.Subscriber("/kinova/error_ack", Empty, self.callback_error_ack)

        # Emergency switch
        rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.callback_emergency_switch)
        self.emergency_switch_pressed = False
        # Dingo lights
        # self.pub_dingo_lights = rospy.Publisher('/cmd_lights', Lights, queue_size=10)

        # Services for setting predefined joint positions
        rospy.Service("/kinova/go_home_position", Trigger, self.handle_go_home_pos)
        rospy.Service("/kinova/go_zero_position", Trigger, self.handle_go_zero_pos)
        # Services for changing control modes
        # rospy.Service("/kinova/change_to_LLC_position", Trigger, self.handle_LLC_position)
        # rospy.Service("/kinova/change_to_LLC_velocity", Trigger, self.handle_LLC_velocity)
        rospy.Service("/kinova/change_to_HLC_position", Trigger, self.handle_HLC_position)
        rospy.Service("/kinova/change_to_HLC_velocity", Trigger, self.handle_HLC_velocity)
        # Gripper
        rospy.Service("/kinova/gripper/open", Trigger, self.handle_gripper_open)
        rospy.Service("/kinova/gripper/close", Trigger, self.handle_gripper_close)


        self.state = State()
        self.different_command_active = False

        """Start the robot """
        with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
            alpha_dq = rospy.get_param("/kinova_feedback/alpha_dq")
            alpha_torque = rospy.get_param("/kinova_feedback/alpha_torque")
            self.kinova = KinovaRobot(router=router, 
                                      real_time_router=real_time_router, 
                                      state = self.state, 
                                      alpha_dq= alpha_dq,
                                      alpha_torque = alpha_torque)
            rospy.loginfo("Current control mode: %s", self.mode)
            self.start_threads()
            self.kinova.start_feedback()



    def start_threads(self):
        spin_thread = Thread(target=self.start_spin_loop)
        spin_thread.start()
    
    def start_spin_loop(self):
        rate = rospy.Rate(PUBLISH_RATE)
        while not rospy.is_shutdown():
            if not self.emergency_switch_pressed:
                if not self.different_command_active:
                    if self.mode == "HLC_position":
                        self.kinova.set_high_level_position(self.state.kinova_command.q)
                    elif self.mode == "HLC_velocity":
                        self.kinova.set_high_level_velocity(self.state.kinova_command.dq)
            else:
                self.set_zero_velocity()
                self.kinova.set_high_level_velocity(self.state.kinova_command.dq)
                
            self.publish_feedback()
            # if np.any(self.kinova.state.kinova_feedback.fault): #Wrong way to read error code
            #     msg_light = Lights()
            #     for i in range(4):
            #         msg_light.lights[i].red = 1.
            #     self.pub_dingo_lights.publish(msg_light)

            rate.sleep()

        if rospy.is_shutdown():
            self.kinova.stop_feedback()
            self.kinova.stop_arm()
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
        # Gripper state
        name = "right_finger_bottom_joint"
        js_msg.name.append(name)
        js_msg.position.append(self.state.kinova_feedback.gripper)
        js_msg.velocity.append(0.0)
        js_msg.effort.append(0.0)

        self.pub_feedback.publish(js_msg)        

    def callback_command(self, msg):
        if not self.emergency_switch_pressed:
            if self.mode == "HLC_position":
                self.state.kinova_command.q = np.rad2deg(msg.data)
            elif self.mode == "HLC_velocity":
                self.state.kinova_command.dq = np.rad2deg(msg.data)
            elif self.mode == "LLC_velocity":   #TODO: not working
                print("Not yet implemented. Command ignored.")

            elif self.mode == "LLC_position":
                print("Not yet implemented. Command ignored.")
    
    def callback_emergency_switch(self, msg):
        if msg.buttons[4]:
            self.emergency_switch_pressed = True
        else:
            self.emergency_switch_pressed = False


    def callback_error_ack(self, msg):
        self.kinova.clear_faults()

    
    def handle_go_home_pos(self, req):
        self.different_command_active = True
        self.state.kinova_command.q = [0, -16, 75, 0, -60, 0]
        self.state.kinova_command.dq = self.kinova.actuator_count * [0.]
        success = self.kinova.set_high_level_position(self.state.kinova_command.q)
        self.different_command_active = False
        return success, "Home position reached"
    
    def handle_go_zero_pos(self, req):
        self.different_command_active = True
        self.state.kinova_command.q = self.kinova.actuator_count * [0.]
        self.state.kinova_command.dq = self.kinova.actuator_count * [0.]
        success = self.kinova.set_high_level_position(self.state.kinova_command.q)
        self.different_command_active = False
        return success, "Zero position reached"

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

    def handle_gripper_open(self, req):
        self.different_command_active = True
        self.kinova.stop_arm()
        success, msg = self.kinova.gripper_open(), "Gripper is opened"
        self.different_command_active = False
        return success, msg

    def handle_gripper_close(self, req):
        self.different_command_active = True
        self.kinova.stop_arm()
        success, msg =  self.kinova.gripper_close(), "Gripper is closed"
        self.different_command_active = False
        return success, msg



if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("kinova_driver")

    kinova_driver = ControlInterface("HLC_velocity")