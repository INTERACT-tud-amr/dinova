#!/usr/bin/env python3

import rospy
import sys
import numpy as np
import casadi as ca
from scipy.spatial.transform import Rotation as R
from forwardkinematics import GenericURDFFk
import os
import shutil

class RobotModel():
    def __init__(self, robot_urdf:str, robot_name:str, root_link:str, end_link:str, lidar:bool, robot_type:str):
        self._lidar = lidar
        self._robot_name = robot_name
        self._robot_type = robot_type
        self._load_model(robot_urdf, root_link, end_link)

    def _load_model(self, robot_urdf:str, root_link:str, end_link:str):
        self._robot_urdf = robot_urdf

        self._robot_fk = GenericURDFFk(
                            self._robot_urdf,
                            root_link = root_link,
                            end_links= end_link
                        )
        self._n_dof = self._robot_fk.n()
        self._end_link = end_link
        self._root_link = root_link
        self._read_link_names(end_link)

    def _read_link_names(self, end_link):
        self._link_names = self._robot_fk.robot._link_names


    def generate_symbolic(self):
        q_ca = ca.SX.sym("q", self._n_dof)

        fk_casadi = []
        for link in self._link_names:
            fk_casadi.append(self._robot_fk.casadi(q_ca, link, position_only=False))

        #DO NOT CHANGE THE NAME OF THE FUNCTION
        fk_casadi_funct = ca.Function('fk_func', [q_ca], fk_casadi) 
        
        if self._lidar:
            FK_FILE_NAME = "fk_" + self._robot_name + "_"+ self._robot_type +"_lidar.cpp" 
        else:
            FK_FILE_NAME = "fk_" + self._robot_name + "_"+ self._robot_type +".cpp"
        gen = ca.CodeGenerator(FK_FILE_NAME)
        gen.add(fk_casadi_funct)
        gen.generate()

        current_script_dir = os.path.dirname(os.path.abspath(__file__))
        CONTROLLER_FOLDER_NEW = os.path.normpath(os.path.join(current_script_dir, "../src/autogen/", FK_FILE_NAME))
        shutil.move(FK_FILE_NAME, CONTROLLER_FOLDER_NEW)

        print(f"\033[92mFK function exported to: {CONTROLLER_FOLDER_NEW}\033[0m")

    



if __name__ == "__main__":
    # Ros initialization
    rospy.init_node("generate_fk")
    robot = RobotModel(robot_urdf = rospy.get_param("dinova_fk_description"),
                       robot_name = rospy.get_param("robot_name"),
                       root_link = rospy.get_param("root_link"),
                       end_link = rospy.get_param("end_link"),
                       lidar = rospy.get_param("lidar"),
                       robot_type = rospy.get_param("robot_type"),
                       )
    robot.generate_symbolic()
