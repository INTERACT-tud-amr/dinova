#!/usr/bin/env python3

import rospy
import ctypes
import os 
import rospkg
import numpy as np
from forwardkinematics import GenericURDFFk
from geometry_msgs.msg import PoseStamped, Pose
from scipy.spatial.transform import Rotation as R

class FK_Autogen():
    def __init__(self, lib_name):
        self._load_model()
        if lib_name is None:
            raise RuntimeError("No FK shared library loaded.")
        self._lib_path = self._get_shared_library_path(lib_name)
        self._fk_lib = self._load_library(self._lib_path)
        print(f"\033[92mFK function imported from: {self._lib_path}\033[0m")


    def _read_link_names(self):
        self._link_names = self._robot_fk.robot._link_names
        self._num_links = len(self._link_names)

    def _load_model(self):
        self._robot_urdf = rospy.get_param("dinova_fk_description")
        self._root_link = rospy.get_param("root_link")
        self._end_link = rospy.get_param("end_link")
   
        self._robot_fk = GenericURDFFk(
                            self._robot_urdf,
                            root_link = self._root_link,
                            end_links= self._end_link
                        )
        
        self._num_dof = self._robot_fk.n()
        self._read_link_names()

    def _find_workspace_path(self):
        current = os.path.dirname(os.path.abspath(__file__))
        while current != os.path.dirname(current):  # Stop at filesystem root
            if os.path.isdir(os.path.join(current, 'devel')):
                return current
            current = os.path.dirname(current)
        raise FileNotFoundError("Could not find ROS workspace (no 'devel' folder found)")

    def _get_shared_library_path(self, lib_name):
        rospack = rospkg.RosPack()
        workspace_path = self._find_workspace_path()
        lib_path = os.path.join(workspace_path, 'devel', 'lib', lib_name)
        return os.path.normpath(lib_path)

    def _load_library(self, lib_path):
        lib = ctypes.CDLL(lib_path) 

        # Define the casadi_f0 function signature
        lib.fk_func.argtypes = [
            ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),  # const casadi_real** arg
            ctypes.POINTER(ctypes.POINTER(ctypes.c_double)),  # casadi_real** res
            ctypes.POINTER(ctypes.c_int),                    # casadi_int* iw
            ctypes.POINTER(ctypes.c_double),                 # casadi_real* w
            ctypes.c_int                                     # int mem
        ]   
        lib.fk_func.restype = ctypes.c_int  # The return type

        self._setting_0 = np.array([0, 0], dtype=np.int32)
        self._setting_1 = np.array([0.0, 0.0], dtype=np.float64)
        self._setting_2 = 0

        self._setting_0 = self._setting_0.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
        self._setting_1 = self._setting_1.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        return lib

    def compute_fk(self, q:np.array, z_compensation=0.0, T_W_vicon=None) -> dict:
        T_W_links = self._compute_fk(q)

        T_W_dict = {}
        print(T_W_vicon)

        if T_W_vicon is not None:
            for idx, T_link in enumerate(T_W_links):
                T_W_link_compen = np.dot(T_W_vicon, T_link)
                link_pose = Pose()
                link_pose.position.x = T_W_link_compen[0,3]
                link_pose.position.y = T_W_link_compen[1,3]
                link_pose.position.z = T_W_link_compen[2,3] + z_compensation
                quat = R.from_matrix(T_W_link_compen[:3, :3]).as_quat()
                link_pose.orientation.x = quat[0]
                link_pose.orientation.y = quat[1]
                link_pose.orientation.z = quat[2]
                link_pose.orientation.w = quat[3]
                T_W_dict[self._link_names[idx]] = link_pose
        else:
            for idx, T_link in enumerate(T_W_links):
                link_pose = Pose()
                link_pose.position.x = T_link[0,3]
                link_pose.position.y = T_link[1,3]
                link_pose.position.z = T_link[2,3] + z_compensation
                quat = R.from_matrix(T_link[:3, :3]).as_quat()
                link_pose.orientation.x = quat[0]
                link_pose.orientation.y = quat[1]
                link_pose.orientation.z = quat[2]
                link_pose.orientation.w = quat[3]
                T_W_dict[self._link_names[idx]] = link_pose

        return T_W_dict

    def _compute_fk(self, q:np.array):
        input_data = (ctypes.POINTER(ctypes.c_double) * 1)(
            q.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        )

        res_data = [np.zeros((16,), dtype=np.float64) for _ in range(self._num_links)]

        output_data = (ctypes.POINTER(ctypes.c_double) * self._num_links)(
            *[r.ctypes.data_as(ctypes.POINTER(ctypes.c_double)) for r in res_data]
        )

        # Call the C function via ctypes
        err_code = self._fk_lib.fk_func(
            input_data,
            output_data,
            self._setting_0, 
            self._setting_1,
            self._setting_2
        )

        if err_code != 0:
            raise RuntimeError(f"FK function failed with code {err_code}")

        transforms = np.array([r.reshape((4, 4)).T for r in res_data])
        return transforms

    def get_link_names(self):
        return self._link_names

    def get_endeffector_name(self):
        return self._end_link

    
