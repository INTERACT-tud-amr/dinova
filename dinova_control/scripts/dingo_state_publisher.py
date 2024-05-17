#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from threading import Thread
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import PoseStamped, Pose

class DingoStatePublisher():
    def __init__(self) -> None:

        self.pub_dinova_omni_state = rospy.Publisher('/dingo/omni_states', JointState, queue_size=1) #TODO

        if rospy.get_param("/vicon/use_vicon"):
            self.pub_dinova_omni_state_vicon = rospy.Publisher('/dingo/omni_states_vicon', JointState, queue_size=1) #TODO
            rospy.Subscriber(rospy.get_param("/vicon/dingo_topic"), PoseStamped, self.callback_vicon)


        rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odometry)

        self.n_dofs_dingo_wheels = 4
        self.n_dofs_dingo_omni = 3
        self.dingo_joint_names = ["front_left_wheel", "front_right_wheel",
                                 "rear_left_wheel", "rear_right_wheel"]
        self.dingo_omni_names = ["omni_joint_x", "omni_joint_y", "omni_joint_theta"]
        self.dingo_base_state = self.init_dingo_joint_state_var()
        self.dingo_vicon_state = self.init_dingo_joint_state_var()
    
    def callback_vicon(self, msg):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        # Dingo
        js_msg.name.append("omni_joint_x")
        js_msg.position.append(msg.pose.position.x)
        js_msg.velocity.append(self.dingo_base_state.velocity[0])
        js_msg.effort.append(0.0)
         # omni_joint_y
        js_msg.name.append("omni_joint_y")
        js_msg.position.append(msg.pose.position.y)
        js_msg.velocity.append(self.dingo_base_state.velocity[1])
        js_msg.effort.append(0.0)
        # omni_joint_theta
        js_msg.name.append("omni_joint_theta")
        theta = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,
                                                          msg.pose.orientation.y,
                                                          msg.pose.orientation.z,
                                                          msg.pose.orientation.w])[2]
    
        js_msg.position.append(theta)
        js_msg.velocity.append(self.dingo_base_state.velocity[2])
        js_msg.effort.append(0.0)
        self.pub_dinova_omni_state_vicon.publish(js_msg)
    
    def callback_odometry(self, msg):
        self.publish_omni_dinova_state(msg)

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
        self.dingo_base_state = js_msg

        self.pub_dinova_omni_state.publish(js_msg)
    
    def init_dingo_joint_state_var(self):
        jointstate = JointState()
        jointstate.header.stamp = rospy.Time.now()
        for i in range(self.n_dofs_dingo_omni):
            jointstate.name.append(self.dingo_omni_names[i])
            jointstate.position.append(0.0)
            jointstate.velocity.append(0.0)
            jointstate.effort.append(0.0)
        return jointstate



if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("dingo_state_publisher")
    dinova_state = DingoStatePublisher()
    rospy.spin()
