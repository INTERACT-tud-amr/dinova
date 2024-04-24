#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from threading import Thread
from nav_msgs.msg import Odometry
import tf
from dinova_control.state import State


class DinovaStatePublisher():
    def __init__(self) -> None:
        self.vicon_use = rospy.get_param("/dingo_feedback/vicon_used")

        self.pub_dinova_state = rospy.Publisher('/dinova/joint_states', JointState, queue_size=1)
        self.pub_dinova_omni_state = rospy.Publisher('/dinova/omni_states', JointState, queue_size=1) #TODO
        if self.vicon_use:
            rospy.Subscriber("/robot_ekf/odometry", Odometry, self.callback_odometry)
        else:
            rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odometry)

        rospy.Subscriber("/joint_states", JointState, self.callback_dingo_state)
        rospy.Subscriber("/kinova/joint_states", JointState, self.callback_kinova_state)

        self.n_dofs_dingo = 4
        self.n_dofs_kinova = 6
        self.dingo_joint_names = ["front_left_wheel", "front_right_wheel",
                                 "rear_left_wheel", "rear_right_wheel"]
        self.kinova_act_state = State()


    def callback_dingo_state(self, msg):
        # publish /dinova/joint_states
        self.publish_joint_states(msg)
    
    def callback_odometry(self, msg):
        # publish [x,y,theta, q]
        self.publish_omni_dinova_state(msg)
    
    def publish_joint_states(self, msg):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        # Dingo
        for i in range(self.n_dofs_dingo):
            name = self.dingo_joint_names[i]
            js_msg.name.append(name)
            js_msg.position.append(msg.position[i])
            js_msg.velocity.append(msg.velocity[i])
            js_msg.effort.append(msg.effort[i])
        # Kinova
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

    def publish_omni_dinova_state(self, msg):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        # Omni base
        # omni_joint_x
        js_msg.name.append("omni_joint_x")
        js_msg.position.append(msg.pose.pose.position.x)
        js_msg.velocity.append(msg.twist.twist.linear.x)
        js_msg.effort.append(0.0)
        # omni_joint_y
        js_msg.name.append("omni_joint_y")
        js_msg.position.append(msg.pose.pose.position.y)
        js_msg.velocity.append(msg.twist.twist.linear.y)
        js_msg.effort.append(0.0)
        # omni_joint_theta
        js_msg.name.append("omni_joint_theta")
        theta = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                          msg.pose.pose.orientation.y,
                                                          msg.pose.pose.orientation.z,
                                                          msg.pose.pose.orientation.w])[2]
    
        js_msg.position.append(theta)
        js_msg.velocity.append(msg.twist.twist.angular.z)
        js_msg.effort.append(0.0)

        # Kinova
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
        self.pub_dinova_omni_state.publish(js_msg)


    def callback_kinova_state(self, msg):
        self.kinova_act_state = msg


if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("dinova_state_publisher")
    dinova_state = DinovaStatePublisher()
    rospy.spin()
