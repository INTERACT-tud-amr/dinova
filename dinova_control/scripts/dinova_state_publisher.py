#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from threading import Thread


class DinovaStatePublisher():
    def __init__(self) -> None:
        self.pub_dinova_state = rospy.Publisher('/dinova/joint_states', JointState, queue_size=1)
        self.pub_dinova_omni_state = rospy.Publisher('/dinova/omni_state', JointState, queue_size=1) #TODO

        rospy.Subscriber("/joint_states", JointState, self.callback_dingo_state)
        rospy.Subscriber("/kinova/joint_states", JointState, self.callback_kinova_state)

        self.n_dofs_dingo = 4
        self.n_dofs_kinova = 6
        self.dingo_link_names = ["front_left_wheel", "front_right_wheel",
                                 "rear_left_wheel", "rear_right_wheel"]
        self.kinova_act_state = None

      

    def callback_dingo_state(self, msg):
        if self.kinova_act_state != None:
            js_msg = JointState()
            js_msg.header.stamp = rospy.Time.now()
            for i in range(self.n_dofs_dingo):
                name = self.dingo_link_names[i]
                js_msg.name.append(name)
                js_msg.position.append(msg.position[i])
                js_msg.velocity.append(msg.velocity[i])
                js_msg.effort.append(msg.effort[i])
            for i in range(self.n_dofs_kinova):
                name = "joint_" + str(i+1)
                js_msg.name.append(name)
                js_msg.position.append(self.kinova_act_state.position[i])
                js_msg.velocity.append(self.kinova_act_state.velocity[i])
                js_msg.effort.append(self.kinova_act_state.effort[i])
            # Gripper state
            name = "right_finger_bottom_joint"
            js_msg.name.append(name)
            js_msg.position.append(self.kinova_act_state.position[-1])
            js_msg.velocity.append(self.kinova_act_state.velocity[-1])
            js_msg.effort.append(self.kinova_act_state.effort[-1])
            
            self.pub_dinova_state.publish(js_msg)

    def callback_kinova_state(self, msg):
        self.kinova_act_state = msg


if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("dinova_state_publisher")
    dinova_state = DinovaStatePublisher()
    rospy.spin()
