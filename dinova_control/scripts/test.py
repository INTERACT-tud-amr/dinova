#!/usr/bin/env python3
from threading import Thread
import time
import sys
import numpy as np
import rospy


from kortex_api.RouterClient import RouterClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, ActuatorConfig_pb2


from dinova_control.utilities import DeviceConnection
from dinova_control.state import State

PUBLISH_RATE = 40 #Hz
import time

def set_servoing_mode(base):
    actuator_ids = {0: 1, 1: 2, 2: 3, 3: 4, 4: 5, 5: 7}
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    # print(base.GetServoingMode().servoing_mode)
    # for n in range(6):
    #     _id = actuator_ids[n]
    #     actuator_modes.append(self.actuator_config.GetControlMode(_id).control_mode)

def set_high_level_velocity(base, velocity):
    joint_speeds = Base_pb2.JointSpeeds()
    actuator_count = base.GetActuatorCount().count
    del joint_speeds.joint_speeds[:]
    speeds = list(velocity)
    for n, speed in enumerate(speeds):
        joint_speed = joint_speeds.joint_speeds.add()
        joint_speed.joint_identifier = n
        joint_speed.value = speed
        joint_speed.duration = 0
    base.SendJointSpeedsCommand(joint_speeds)

def gripper_open(base):
    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()
    finger.finger_identifier = 1
    gripper_command.mode = Base_pb2.GRIPPER_SPEED
    finger.value = 0.5
    base.SendGripperCommand(gripper_command)

    # Wait for reported position to be opened
    gripper_request = Base_pb2.GripperRequest()
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            if np.abs(gripper_measure.finger[0].value) < 0.01:
                break
        else: # Else, no finger present in answer, end loop
            break

def gripper_close(base):
    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()
    finger.finger_identifier = 1
    gripper_command.mode = Base_pb2.GRIPPER_SPEED
    finger.value = -0.8
    base.SendGripperCommand(gripper_command)

    # Wait for reported speed to be 0
    gripper_request = Base_pb2.GripperRequest()
    gripper_request.mode = Base_pb2.GRIPPER_SPEED
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current speed is : {0}".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value <= 0.00:
                break
        else: # Else, no finger present in answer, end loop
            break


if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("kinova_driver")

    with DeviceConnection.createTcpConnection() as router, DeviceConnection.createUdpConnection() as real_time_router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(real_time_router) #for reading feedback
        rospy.loginfo("Connected to Kinova")
        set_servoing_mode(base)

        # Main loop
        print("Sending velocity 1")
        SPEED = 20
        velocity = [-SPEED, 0.0, 0.0, SPEED, 0.0, 0.0]
        set_high_level_velocity(base, velocity)
        time.sleep(2.5)
        start = time.time()
        base.Stop() # 25ms
        end = time.time()
        print("Elapsed time: ", end-start)
        print("Closing gripper")
        gripper_close(base)
        print("Sending velocity 2")
        velocity = [SPEED, 0.0, 0.0, -SPEED, 0.0, 0.0]
        set_high_level_velocity(base, velocity)
        time.sleep(2.5)
        base.Stop()
        print("Opening gripper")
        gripper_open(base)

        print("Sending velocity 3")
        velocity = [0, -SPEED, 0.0, 0, 0.0, 0.0]
        set_high_level_velocity(base, velocity)
        time.sleep(2.5)
        base.Stop()
        rospy.loginfo("Stopping arm")


