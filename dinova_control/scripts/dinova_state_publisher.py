#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from threading import Thread
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import PoseStamped, Pose
import time
import copy

class DinovaStatePublisher():
    def __init__(self) -> None:
        self.pub_dinova_state = rospy.Publisher('dinova/joint_states', JointState, queue_size=1)
        self.pub_dinova_omni_state = rospy.Publisher('dinova/omni_states', JointState, queue_size=1) #TODO

        if rospy.get_param("vicon/use_vicon"):
            self.pub_dinova_omni_state_vicon = rospy.Publisher('dinova/omni_states_vicon', JointState, queue_size=1) #TODO
            rospy.Subscriber(rospy.get_param("vicon/dingo_topic"), PoseStamped, self.callback_vicon)


        rospy.Subscriber("odometry/filtered", Odometry, self.callback_odometry)

        rospy.Subscriber("joint_states", JointState, self.callback_dingo_state)
        rospy.Subscriber("kinova/joint_states", JointState, self.callback_kinova_state)

        self.n_dofs_dingo_wheels = 4
        self.n_dofs_dingo_omni = 3
        self.n_dofs_kinova = 6
        self.dingo_joint_names = ["front_left_wheel", "front_right_wheel",
                                 "rear_left_wheel", "rear_right_wheel"]
        self.dingo_omni_names = ["omni_joint_x", "omni_joint_y", "omni_joint_theta"]
        self.kinova_act_state = self.init_kinova_joint_state_var()
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
        js_msg.velocity = self.generate_velocities_from_vicon(js_msg)[0:3]

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
        self.pub_dinova_omni_state_vicon.publish(js_msg)


    def callback_dingo_state(self, msg):
        if self.kinova_act_state != None:
            # publish /dinova/joint_states
            self.publish_joint_states(msg)
    
    def callback_odometry(self, msg):
        if self.kinova_act_state != None:
            # publish [x,y,theta, q]
            self.publish_omni_dinova_state(msg)
    
    def publish_joint_states(self, msg):
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        # Dingo
        for i in range(self.n_dofs_dingo_wheels):
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
        self.dingo_base_state = js_msg

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
        
    def generate_velocities_from_vicon(self, js_msg):
        if self.position_previous == []:
            self.position_previous = js_msg.position
            self.time_previous = time.perf_counter()
            return list(self.base_vel)
        
        time_current = time.perf_counter()
        vel_current = (np.array(js_msg.position[:3]) - np.array(self.position_previous[:3]))/(time_current - self.time_previous)
        self.base_vel = self.alpha_vel*vel_current + (1-self.alpha_vel)*copy.deepcopy(self.base_vel)
        self.position_previous = js_msg.position[:3]
        self.time_previous = copy.deepcopy(time_current)
        return list(self.base_vel)
        

    def callback_kinova_state(self, msg):
        self.kinova_act_state = msg
    
    def init_dingo_joint_state_var(self):
        jointstate = JointState()
        jointstate.header.stamp = rospy.Time.now()
        for i in range(self.n_dofs_dingo_omni):
            jointstate.name.append(self.dingo_omni_names[i])
            jointstate.position.append(0.0)
            jointstate.velocity.append(0.0)
            jointstate.effort.append(0.0)
        return jointstate
    
    def init_kinova_joint_state_var(self):
        jointstate = JointState()
        jointstate.header.stamp = rospy.Time.now()
        for i in range(self.n_dofs_kinova):
            name = "joint_" + str(i+1)
            jointstate.name.append(name)
            jointstate.position.append(0.0)
            jointstate.velocity.append(0.0)
            jointstate.effort.append(0.0)
        jointstate.name.append("right_finger_bottom_joint")
        jointstate.position.append(0.0)
        jointstate.velocity.append(0.0)
        jointstate.effort.append(0.0)
        return jointstate



if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("dinova_state_publisher")
    dinova_state = DinovaStatePublisher()
    rospy.spin()
