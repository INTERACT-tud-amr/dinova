from kortex_api.RouterClient import RouterClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, ActuatorConfig_pb2


import numpy as np
from threading import Thread, Event
import time
from typing import Literal

from .specifications import Position, actuator_ids, ranges
from .state import State


class KinovaRobot():
    def __init__(self, 
                 base: BaseClient = None,
                 base_cyclic: BaseCyclicClient = None,
                 router: RouterClient = None,
                 real_time_router: RouterClient = None,
                 actuator_config: ActuatorConfigClient = None,
                 state: State = None,
                 alpha_dq: float = 0.7,
                 alpha_torque: float = 0.7) -> None:
        
        if None in [base, base_cyclic, actuator_config]:
            self.base = BaseClient(router)
            self.base_cyclic = BaseCyclicClient(real_time_router)
            self.actuator_config = ActuatorConfigClient(router)
        else:
            self.base = base
            self.base_cyclic = base_cyclic
            self.actuator_config = actuator_config

        self.actuator_count = self.base.GetActuatorCount().count
        self.state = state
        
        # High-level control "HLC" (40Hz) or Low-level control "LLC" (1000Hz)
        self.mode = "HLC"
        # Set Servoing Mode
        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING) # A second client is attempting to control the robot while it is in single-level servoing mode

        # Refresh
        self.controller_connected = False
        self.refresh_feedback()

        # Initialize commands
        self._initialize_command()
        self.joints_command = np.zeros(self.actuator_count)

        # First-order filter
        self.dq_old = np.zeros(self.actuator_count)
        self.alpha_dq = alpha_dq
        self.torque_old = np.zeros(self.actuator_count)
        self.alpha_torque = alpha_torque

        # self.rate_counter = RateCounter(1000)

    def clear_faults(self) -> None:
        """Clear the faults."""
        self.base.ClearFaults()

    def get_position(self, joint: int) -> float:
        position = getattr(self.feedback.actuators[joint], "position")
        lower_bound = ranges["position"][joint][0]
        upper_bound = ranges["position"][joint][1]
        position -= 360 if position > upper_bound else 0

        return np.deg2rad(position)
    

    def get_velocity(self, joint: int) -> float:
        velocity = getattr(self.feedback.actuators[joint], "velocity")
        return np.deg2rad(velocity)

    def get_torque(self, joint: int) -> float:
        """Get the torque of a joint."""
        return getattr(self.feedback.actuators[joint], "torque")

    def get_gripper_position(self) -> float:
        """Get the gripper position."""
        return (self.feedback.interconnect.gripper_feedback.motor[0].position) / 100
    

    def update_state(self) -> None:
        for n in range(self.actuator_count):
            # Read position
            self.state.kinova_feedback.q[n] = self.get_position(n)
            # Read velocity
            dq_new = self.get_velocity(n)
            self.state.kinova_feedback.dq[n] = self.alpha_dq*dq_new + (1- self.alpha_dq)*self.dq_old[n]
            self.dq_old[n] = dq_new
            # Read torque
            torque_new = self.get_torque(n)
            self.state.kinova_feedback.torque[n] = self.alpha_torque*torque_new + (1- self.alpha_torque)*self.torque_old[n]
            self.torque_old[n] = torque_new
            # Read gripper pos
            self.state.kinova_feedback.gripper = self.get_gripper_position()
            # Read error
            self.state.kinova_feedback.fault[n] = self.feedback.actuators[n].fault_bank_a

    def refresh_feedback(self) -> None:
        """Refresh."""
        if not self.changing_servoing_mode and self.controller_connected:
            try:
                self.feedback = self.base_cyclic.Refresh(self.command)
            except Exception as e:
                print(f"Exception: {e}")
                self.controller_connected = False
        else:
            self.feedback = self.base_cyclic.RefreshFeedback()

    def start_feedback_in_new_thread(self) -> None:
        thread = Thread(target=self.start_feedback)
        thread.start()

    def start_feedback(self) -> None:
        self.active = True
        self._refresh_loop()

    def stop_feedback(self, *args: any) -> None:
        """Stop the update loop."""
        print("Closing feedback loop with arm...")
        self.active = False

    def _refresh_loop(self) -> None:
        while self.active:
            self.refresh_feedback()
            self.update_state()
            if self.mode == "LLC_task":
                if not np.any(self.state.kinova_feedback.fault):
                    self.set_command(self.joints_command)
                else:
                    print("not implemented")
    

    def _set_servoing_mode(self, value: int) -> None:
        """Set the servoing mode of the robot."""
        self.changing_servoing_mode = True
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = value
        self.base.SetServoingMode(base_servo_mode)
        self._update_modes()    #default is position mode
        self.changing_servoing_mode = False

    def _update_modes(self) -> None:
        """Update the modes."""
        self.servoing_mode = self.base.GetServoingMode().servoing_mode
        actuator_modes = []
        for n in range(self.actuator_count):
            _id = actuator_ids[n]
            actuator_modes.append(self.actuator_config.GetControlMode(_id).control_mode)
        self.actuator_modes = actuator_modes

    def set_control_mode(
        self, joint: int, mode: Literal["position", "velocity", "current"]
    ) -> None:
            """Set the control mode of an actuator."""
            mode = getattr(ActuatorConfig_pb2, mode.upper())
            control_mode_information = ActuatorConfig_pb2.ControlModeInformation()
            control_mode_information.control_mode = mode
            _id = actuator_ids[joint]
            self.actuator_config.SetControlMode(control_mode_information, _id)
            self._update_modes()

    def _initialize_command(self) -> None:
        self.command = BaseCyclic_pb2.Command()
        for n in range(self.actuator_count):
            actuator_command = BaseCyclic_pb2.ActuatorCommand()
            actuator_command.flags = 1
            actuator_command.position = self.feedback.actuators[n].position
            actuator_command.velocity = self.feedback.actuators[n].velocity
            self.command.actuators.extend([actuator_command])

    def _move_gripper(self, pos_msg) -> None:
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        pos = self.get_gripper_position() + pos_msg
        if pos >= 0.0 and pos <= 0.8:
            finger.value = pos
            self.base.SendGripperCommand(gripper_command)
        else:
            print("Invalid command for gripper")

    def open_gripper(self) -> bool:
        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.finger_identifier = 1

        # Set speed to open gripper
        finger.value = 1.0
        self.base.SendGripperCommand(gripper_command)
        # Wait for reported position to be opened
        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                if gripper_measure.finger[0].value < 0.01:
                    break
            else: # Else, no finger present in answer, end loop
                break
        return True


    def close_gripper(self) -> None:
         # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()
        gripper_command.mode = Base_pb2.GRIPPER_SPEED
        finger.finger_identifier = 1

        finger.value = -1.0
        self.base.SendGripperCommand(gripper_command)
        # Wait for reported position to be opened
        gripper_request = Base_pb2.GripperRequest()
        gripper_request.mode = Base_pb2.GRIPPER_POSITION
        while True:
            gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
            if len (gripper_measure.finger):
                if gripper_measure.finger[0].value >= 1:
                    break
            else: # Else, no finger present in answer, end loop
                break
        return True

    def set_high_level_position(self, position) -> None:
        """Perform a high level move."""
        action = Base_pb2.Action()
        action.name = ""
        action.application_data = ""

        for n, pos in enumerate(position):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = n
            joint_angle.value = pos

        return self._execute_action(action)

    def set_high_level_velocity(self, velocity) -> None:
        joint_speeds = Base_pb2.JointSpeeds()
        speeds = list(velocity)
        if not np.any(self.state.kinova_feedback.fault):
            for n, speed in enumerate(speeds):
                joint_speed = joint_speeds.joint_speeds.add()
                joint_speed.joint_identifier = n
                joint_speed.value = speed
                joint_speed.duration = 0
            start = time.time()
            self.base.SendJointSpeedsCommand(joint_speeds)
        else:
            pass

    def _execute_action(self, action: Base_pb2.Action = None) -> bool:
        event = Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self._check_for_end_or_abort(event), Base_pb2.NotificationOptions()
        )

        self.base.ExecuteAction(action)
        finished = event.wait(30)
        self.base.Unsubscribe(notification_handle)

        if finished:
            success = True
        else:
            success = False
        return success

    def _check_for_end_or_abort(self, event: Event) -> callable:
        """Return a closure checking for END or ABORT notifications."""

        def check(notif: Base_pb2.ActionNotification, event: Event = event) -> None:
            if notif.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
                event.set()

        return check


    def copy_feedback_to_command(self, joint: int = None) -> None:
        """Copy the feedback to the command message."""
        for prop in ["position", "velocity", "current_motor"]:
            for n in range(self.actuator_count) if joint is None else [joint]:
                value = getattr(self.feedback.actuators[n], prop)
                setattr(self.command.actuators[n], prop, value)

    def start_LLC(self) -> None:
        """Start low_level control and set the position mode for safety"""
        self.copy_feedback_to_command()
        for n in range(self.actuator_count):
            self.set_control_mode(n, "position")
        self._set_servoing_mode(Base_pb2.LOW_LEVEL_SERVOING)
        self.mode = "LLC"

    def connect_LLC(self) -> None:
        """Connect a controller to the LLC of the robot."""
        # self.state.controller.start_control_loop()
        for n in range(self.actuator_count):
            self.copy_feedback_to_command()
            self.base_cyclic.Refresh(self.command)
            self.set_control_mode(n, "velocity")
        self.controller_connected = True
        self.mode = "LLC_task"

    
    def disconnect_LLC(self) -> None:
        """Disconnect a controller from the LLC of the robot."""
        self.controller_connected = False
        for joint in range(self.actuator_count):
            self.set_control_mode(joint, "position")
        # self.state.controller.stop_control_loop() #TODO
        self.mode = "LLC"


    def stop_LLC(self) -> None:
        """Stop low_level control."""
        self._set_servoing_mode(Base_pb2.SINGLE_LEVEL_SERVOING)
        self.mode = "HLC"
        return True, "Low_level control disabled."

    def set_command(self, commands: list) -> None:
        """Set the command."""
        for n, command in enumerate(commands):
            self.copy_feedback_to_command(n)
            self.command.actuators[n].velocity = command


    def get_control_modes(self):
        """Get the control mode of an actuator."""
        return [
            ActuatorConfig_pb2.ControlMode.Name(self.actuator_modes[n])
            for n in range(self.actuator_count)
        ]

    def get_servoing_mode(self):
        """Get the servoing mode of the robot."""
        return Base_pb2.ServoingMode.Name(self.servoing_mode)