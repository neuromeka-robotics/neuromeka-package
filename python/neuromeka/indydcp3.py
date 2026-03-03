import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *

from neuromeka.common import *
from neuromeka.enums import *

import time
from typing import Optional, List

import grpc
from google.protobuf import json_format
from google.protobuf.json_format import ParseDict

from neuromeka.indydcp3_channels import (
    HelpersMixin,
    BootMixin,
    RTDEMixin,
    DeviceMixin,
    CRIMixin,
    ControlMixin,
    ConfigMixin,
)

CONTROL_SOCKET_PORT = [20001, 30001]
DEVICE_SOCKET_PORT = [20002, 30002]
CONFIG_SOCKET_PORT = [20003, 30003]
RTDE_SOCKET_PORT = [20004, 30004]
BOOT_SOCKET_PORT = [20010, 30010]
CRI_SOCKET_PORT = [20181, 30181]


class IndyDCP3(
    HelpersMixin,
    BootMixin,
    RTDEMixin,
    DeviceMixin,
    CRIMixin,
    ControlMixin,
    ConfigMixin,
):
    def __init__(self, robot_ip='127.0.0.1', index=0):
        if index not in [0, 1]:
            raise ValueError("Index must be 0 or 1")

        self.boot_channel = grpc.insecure_channel('{}:{}'.format(robot_ip, BOOT_SOCKET_PORT[index]))
        self.control_channel = grpc.insecure_channel('{}:{}'.format(robot_ip, CONTROL_SOCKET_PORT[index]))
        self.device_channel = grpc.insecure_channel('{}:{}'.format(robot_ip, DEVICE_SOCKET_PORT[index]))
        self.config_channel = grpc.insecure_channel('{}:{}'.format(robot_ip, CONFIG_SOCKET_PORT[index]))
        self.rtde_channel = grpc.insecure_channel('{}:{}'.format(robot_ip, RTDE_SOCKET_PORT[index]))
        self.cri_channel = grpc.insecure_channel('{}:{}'.format(robot_ip, CRI_SOCKET_PORT[index]))

        self.boot = BootStub(self.boot_channel)
        self.control = ControlStub(self.control_channel)
        self.device = DeviceStub(self.device_channel)
        self.config = ConfigStub(self.config_channel)
        self.rtde = RTDataExchangeStub(self.rtde_channel)
        self.cri = CRIStub(self.cri_channel)

        self._joint_waypoint = []
        self._task_waypoint = []

    def __del__(self):
        if self.boot_channel is not None:
            self.boot_channel.close()
        if self.control_channel is not None:
            self.control_channel.close()
        if self.device_channel is not None:
            self.device_channel.close()
        if self.config_channel is not None:
            self.config_channel.close()
        if self.rtde_channel is not None:
            self.rtde_channel.close()
        if self.cri_channel is not None:
            self.cri_channel.close()

    ############################
    # Cross-channel utility methods
    ############################
    def get_control_data(self):
        return self.get_robot_data()

    def add_joint_waypoint(self, waypoint: list):
        self._joint_waypoint.append(waypoint)
        return True

    def get_joint_waypoint(self):
        return self._joint_waypoint

    def clear_joint_waypoint(self):
        self._joint_waypoint.clear()
        return True

    def move_joint_waypoint(self, move_time=None):
        for wp in self._joint_waypoint:
            if move_time is None:
                self.movej(jtarget=wp, blending_type=BlendingType.OVERRIDE)
            else:
                self.movej_time(jtarget=wp, blending_type=BlendingType.OVERRIDE, move_time=move_time)
            self.wait_progress(progress=100)
        return True

    def add_task_waypoint(self, waypoint: list):
        self._task_waypoint.append(waypoint)
        return True

    def get_task_waypoint(self):
        return self._task_waypoint

    def clear_task_waypoint(self):
        self._task_waypoint.clear()
        return True

    def move_task_waypoint(self, move_time=None):
        for wp in self._task_waypoint:
            if move_time is None:
                self.movel(ttarget=wp, blending_type=BlendingType.OVERRIDE)
            else:
                self.movel_time(ttarget=wp, blending_type=BlendingType.OVERRIDE, move_time=move_time)
            self.wait_progress(progress=100)
        return True

    def move_home(self):
        home_pos = self.get_home_pos()['jpos']
        self.movej(home_pos,
                   blending_type=BlendingType.NONE,
                   base_type=JointBaseType.ABSOLUTE,
                   blending_radius=0.0,
                   vel_ratio=Limits.JogVelRatioDefault,
                   acc_ratio=Limits.JogAccRatioDefault,
                   post_condition=PostCondition(),
                   teaching_mode=False)

    def start_log(self):
        """
        Start realtime data logging
        """
        int_vars_to_set = [{"addr": 300, "value": 1}]
        self.set_int_variable(int_vars_to_set)

    def end_log(self):
        """
        Finish realtime data logging and save the realtime data in STEP
        saved path:
            /home/user/release/IndyDeployments/RTlog/RTLog.csv
        """
        int_vars_to_set = [{"addr": 300, "value": 2}]
        self.set_int_variable(int_vars_to_set)

    def wait_for_operation_state(self, wait_op_state=None):
        if wait_op_state is not None:
            while self.get_robot_data()['op_state'] != wait_op_state:
                time.sleep(0.01)

    def wait_for_motion_state(self, wait_motion_state=None):
        motion_list = ["is_in_motion", "is_target_reached", "is_pausing", "is_stopping", "has_motion"]
        if wait_motion_state is not None and wait_motion_state in motion_list:
            while self.get_motion_data()[wait_motion_state] is False:
                time.sleep(0.01)
