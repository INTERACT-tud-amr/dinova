import numpy as np
from dataclasses import dataclass

JOINTS = 6

@dataclass
class JointData:
    """Dataclass containing the position, velocity and current/torque of joints."""

    n: int

    def __post_init__(self) -> None:
        """Initialize vectors."""
        self.q = np.zeros(self.n)
        self.dq = np.zeros(self.n)
        self.torque = np.zeros(self.n)
        self.fault = np.zeros(self.n)
        self.gripper = 0.0


class State:
    def __init__(self) -> None:
        self.kinova_feedback = JointData(JOINTS)
        self.kinova_command = JointData(JOINTS)

    def __str__(self):
        # msg = "===============\n" \
        #     + "Kinova Command:\n" \
        #     + "\tNot implemented.\n" \
        #     + "---\n" \
        #     +"Kinova Feedback:\n"  \
        #     +"Joints position:\n{q}\n" \
        #     +"Joints velocity:\n{dq}\n" \
        #     +"Joints faults:\n{faults}"
        msg = "===============\n" \
            +"Kinova Feedback:\n"  \
            +"---\n" \
            +"Joints position:\n{q}\n" \
            +"Joints velocity:\n{dq}\n" \
            +"Joints faults:\n{faults}"
        return msg.format(q=self.kinova_feedback.q, dq=self.kinova_feedback.dq, faults = self.kinova_feedback.fault)

