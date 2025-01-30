#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from threading import Thread
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import time
import copy

class DingoStatePublisher():
    def __init__(self) -> None:

        self.pub_dinova_omni_state = rospy.Publisher('dingo/omni_states', JointState, queue_size=1) #TODO

        if rospy.get_param("vicon/use_vicon"):
            self.pub_dinova_omni_state_vicon = rospy.Publisher('dingo/omni_states_vicon', JointState, queue_size=1) #TODO
            rospy.Subscriber(rospy.get_param("vicon/dingo_topic"), PoseStamped, self.callback_vicon)


        rospy.Subscriber("odometry/filtered", Odometry, self.callback_odometry)

        self.n_dofs_dingo_wheels = 4
        self.n_dofs_dingo_omni = 3
        self.dingo_joint_names = ["front_left_wheel", "front_right_wheel",
                                 "rear_left_wheel", "rear_right_wheel"]
        self.dingo_omni_names = ["omni_joint_x", "omni_joint_y", "omni_joint_theta"]
        self.dingo_base_state = self.init_dingo_joint_state_var()
        self.dingo_vicon_state = self.init_dingo_joint_state_var()
        self.time_previous = 0.
        self.base_vel = np.array([0., 0., 0.])
        self.position_previous = []
        self.alpha_vel = 0.5 # for filtering velocities from vicon
    
    def callback_vicon(self, msg):
        theta = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,
                                                          msg.pose.orientation.y,
                                                          msg.pose.orientation.z,
                                                          msg.pose.orientation.w])[2]
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        velocities_robot_frame = self.dingo_base_state.velocity[0:2]
        rotation = np.array([
            [np.cos(theta), -np.sin(theta)], 
            [np.sin(theta), np.cos(theta)], 
        ])
        velocities_vicon_frame = np.dot(rotation, velocities_robot_frame)
        # Dingo
        js_msg.name.append("omni_joint_x")
        js_msg.position.append(msg.pose.position.x)
        js_msg.velocity.append(velocities_vicon_frame[0])
        js_msg.effort.append(0.0)
         # omni_joint_y
        js_msg.name.append("omni_joint_y")
        js_msg.position.append(msg.pose.position.y)
        js_msg.velocity.append(velocities_vicon_frame[1])
        js_msg.effort.append(0.0)
        # omni_joint_theta
        js_msg.name.append("omni_joint_theta")
    
        js_msg.position.append(theta)
        js_msg.velocity.append(self.dingo_base_state.velocity[2])
        js_msg.effort.append(0.0)
        js_msg.velocity = self.generate_velocities_from_vicon(js_msg)
        self.pub_dinova_omni_state_vicon.publish(js_msg)
        
    def generate_velocities_from_vicon(self, js_msg):
        if self.position_previous == []:
            self.position_previous = js_msg.position
            self.time_previous = time.perf_counter()
            return list(self.base_vel)
        
        time_current = time.perf_counter()
        vel_current = (np.array(js_msg.position) - np.array(self.position_previous))/(time_current - self.time_previous)
        self.base_vel = self.alpha_vel*vel_current + (1-self.alpha_vel)*copy.deepcopy(self.base_vel)
        self.position_previous = js_msg.position
        self.time_previous = copy.deepcopy(time_current)
        return list(self.base_vel)
        
    
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
