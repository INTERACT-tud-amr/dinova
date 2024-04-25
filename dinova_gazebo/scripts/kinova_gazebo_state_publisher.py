#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState


class KinovaGazeboStatePublisher():
    def __init__(self) -> None:
        self.pub_kinova_state = rospy.Publisher('/kinova/joint_states', JointState, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.callback_joint_state)
    
        self.n_dofs_kinova = 6

    def callback_joint_state(self, msg):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        # kinova joints
        for i in range(self.n_dofs_kinova):
            name = "joint_" + str(i+1)
            js_msg.name.append(name)
            js_msg.position.append(msg.position[i])
            js_msg.velocity.append(msg.velocity[i])
            js_msg.effort.append(msg.effort[i])

        self.pub_kinova_state.publish(js_msg)



if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("kinova_gazebo_state_publisher")
    dinova_state = KinovaGazeboStatePublisher()
    rospy.spin()