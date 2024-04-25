#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState


class DinovaGazeboStatePublisher():
    def __init__(self) -> None:
        self.pub_dinova_state = rospy.Publisher('/dinova/omni_states', JointState, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.callback_joint_state)
    
        self.dingo_joint_names = ["omni_joint_x", "omni_joint_y", "omni_joint_theta"]
        self.n_dofs_kinova = 6

    def callback_joint_state(self, msg):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        # omni_joint_x
        js_msg.name.append(self.dingo_joint_names[0])
        js_msg.position.append(msg.position[7])
        js_msg.velocity.append(msg.velocity[7])
        js_msg.effort.append(msg.effort[7])
        # omni_joint_y
        js_msg.name.append(self.dingo_joint_names[1])
        js_msg.position.append(msg.position[8])
        js_msg.velocity.append(msg.velocity[8])
        js_msg.effort.append(msg.effort[8])
        # omni_joint_theta
        js_msg.name.append(self.dingo_joint_names[2])
        js_msg.position.append(msg.position[6])
        js_msg.velocity.append(msg.velocity[6])
        js_msg.effort.append(msg.effort[6])
        # kinova joints
        for i in range(self.n_dofs_kinova):
            name = "joint_" + str(i+1)
            js_msg.name.append(name)
            js_msg.position.append(msg.position[i])
            js_msg.velocity.append(msg.velocity[i])
            js_msg.effort.append(msg.effort[i])
        # gripper state
        js_msg.name.append("right_finger_bottom_joint")
        js_msg.position.append(msg.position[9])
        js_msg.velocity.append(msg.velocity[9])
        js_msg.effort.append(msg.effort[9])

        self.pub_dinova_state.publish(js_msg)



if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("dinova_gazebo_state_publisher")
    dinova_state = DinovaGazeboStatePublisher()
    rospy.spin()