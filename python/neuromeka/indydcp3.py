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

CONTROL_SOCKET_PORT = [20001, 30001]
DEVICE_SOCKET_PORT = [20002, 30002]
CONFIG_SOCKET_PORT = [20003, 30003]
RTDE_SOCKET_PORT = [20004, 30004]
BOOT_SOCKET_PORT = [20010, 30010]
CRI_SOCKET_PORT = [20181, 30181]

class IndyDCP3:
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

    def __to_digital_request_list__(self, digital_signal_list) -> list:
        request_list = []
        if digital_signal_list is not None:
            for signal in digital_signal_list:
                request_list.append(device_msgs.DigitalSignal(address=signal['address'], state=signal['state']))
        return request_list

    def __to_analog_request_list__(self, analog_signal_list) -> list:
        request_list = []
        if analog_signal_list is not None:
            for signal in analog_signal_list:
                if isinstance(signal, (tuple, list)) and len(signal) == 2:
                    address, voltage = signal
                    request_list.append(device_msgs.AnalogSignal(address=address, voltage=voltage))
                elif isinstance(signal, dict):
                    request_list.append(device_msgs.AnalogSignal(address=signal['address'], voltage=signal['voltage']))
                else:
                    request_list.append(signal)
        return request_list

    def __to_endtool_signal_list__(self, endtool_signal_list) -> list:
        request_list = []
        if endtool_signal_list is not None:
            for sig in endtool_signal_list:
                if isinstance(sig, device_msgs.EndtoolSignal):
                    request_list.append(sig)
                elif isinstance(sig, (tuple, list)) and len(sig) == 2:
                    port, states = sig
                    request_list.append(device_msgs.EndtoolSignal(port=str(port), states=list(states)))
                elif isinstance(sig, dict):
                    request_list.append(device_msgs.EndtoolSignal(port=sig['port'], states=list(sig['states'])))
                else:
                    request_list.append(device_msgs.EndtoolSignal(port=sig.port, states=list(sig.states)))
        return request_list

    ############################
    # IndyDCP3 API protocols
    ############################
    def get_robot_data(self):
        """
        Output:
            running_hours -> uint32
            running_mins -> uint32
            running_secs -> uint32
            op_state -> OpState
            sim_mode -> bool
            is_robot_connected -> bool
            q -> float[]
            qdot -> float[]
            p -> float[]
            pdot -> float[]
            ref_frame -> float[]
            tool_frame -> float[]
            tool_link -> int32
            locked_joint -> int32
            response -> {code, msg}
        """
        response = self.rtde.GetControlData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_control_data(self):
        return self.get_robot_data()

    def get_control_state(self):
        """
        Output:
            q -> float[]
            qdot -> float[]
            qddot -> float[]
            qdes -> float[]
            qdotdes -> float[]
            qddotdes -> float[]
            p -> float[]
            pdot -> float[]
            pddot -> float[]
            pdes -> float[]
            pdotdes -> float[]
            pddotdes -> float[]
            tau -> float[]
            tau_act -> float[]
            tau_ext -> float[]
            tau_jts -> float[]
            tau_jts_raw1 -> float[]
            tau_jts_raw2 -> float[]
            manipulability -> float
            response -> {code, msg}
        """
        response = self.rtde.GetControlState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_motion_data(self):
        """
        Output:
            traj_state -> TrajState
            traj_progress -> int32
            is_in_motion -> bool
            is_target_reached -> bool
            is_pausing -> bool
            is_stopping -> bool
            has_motion -> bool
            speed_ratio -> int32
            motion_id -> int32
            remain_distance -> float
            motion_queue_size -> uint32
            cur_traj_progress -> int32
            response -> {code, msg}
        """
        response = self.rtde.GetMotionData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_servo_data(self):
        """
        Output:
            status_codes -> string[]
            temperatures -> float[]
            voltages -> float[]
            currents -> float[]
            torques -> float[]
            servo_actives -> bool[]
            brake_actives -> bool[]
            response -> {code, msg}
        """
        response = self.rtde.GetServoData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_collision_model_state(self):
        """
        Output:
            collisions -> ModelCollision[]
                rule -> ContactRule
                link1 -> int32
                link2 -> int32
                tool_idx -> int32
                env_idx -> int32
        """
        response = self.rtde.GetCollisionModelState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_reserved_data(self):
        """
        Output:
            qres1 -> float[]
            qres2 -> float[]
            qdotres1 -> float[]
            qdotres2 -> float[]
            taures1 -> float[]
            taures2 -> float[]
            eres1 -> float[]
            eres2 -> float[]
            edotres1 -> float[]
            edotres2 -> float[]
            response -> {code, msg}
        """
        response = self.rtde.GetReservedData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    # def test_function(self, request: dict):
    #     req = rtde_msgs.TestRequest()
    #     ParseDict(request, req)
    #     response = self.rtde.TestFunction(req)
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    def get_violation_data(self):
        """
        Output:
            violation_code -> uint64
            j_index -> uint32
            i_args -> int32[]
            f_args -> float[]
            violation_str -> string
            violation_id -> uint64
            response -> {code, msg}
        """
        response = self.rtde.GetViolationData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_violation_message_queue(self):
        """
        Output:
            violation_queue -> ViolationData[]
            response -> {code, msg}
        """
        response = self.rtde.GetViolationMessageQueue(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def commit_violation(self, violation: dict):
        """
        Input:
            violation -> dict {violation_type, stop_category, source, axis_idx, misc_fvalue, misc_ivalue, misc_min, misc_max, misc_text}
        Output:
            response -> {code, msg}
        """
        req = device_msgs.ViolationRequest()
        ParseDict(violation, req)
        response = self.device.CommitViolation(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_rt_task_times(self):
        """
        Output:
            task_times -> NamedFloat[] {name, value} (unit: us)
        """
        response = self.device.GetRTTaskTimes(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    # def socket_cmd_set_config(self, config: dict):
    #     req = device_msgs.SocketCommandConfig()
    #     ParseDict(config, req)
    #     response = self.device.SocketCmdSetConfig(req)
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    # def socket_cmd_get_config(self):
    #     response = self.device.SocketCmdGetConfig(common_msgs.Empty())
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    # def socket_cmd_start(self):
    #     response = self.device.SocketCmdStart(common_msgs.Empty())
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    # def socket_cmd_stop(self):
    #     response = self.device.SocketCmdStop(common_msgs.Empty())
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    # def socket_cmd_send_data(self, payload: dict):
    #     req = device_msgs.SocketPayload()
    #     ParseDict(payload, req)
    #     response = self.device.SocketCmdSendData(req)
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    # def socket_cmd_get_latest_data(self):
    #     response = self.device.SocketCmdGetLatestData(common_msgs.Empty())
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    def get_program_data(self):
        """
        Output:
            program_state -> ProgramState
            cmd_id -> int32
            sub_cmd_id -> int32
            running_hours -> uint32
            running_mins -> uint32
            running_secs -> uint32
            program_name -> string
            program_alarm -> string
            program_annotation -> string
            speed_ratio -> int32
            start_line_index -> {cmd_id, sub_cmd_id}
            end_line_index -> {cmd_id, sub_cmd_id}
            debug_mode -> bool
            response -> {code, msg}
        """
        response = self.rtde.GetProgramData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_stop_state(self):
        """
        Output:
            category -> StopCategory (STOP_CAT_0=0, STOP_CAT_1=1, STOP_CAT_2=2, STOP_CAT_NONE=-1)
            response -> {code, msg}
        """
        response = self.rtde.GetStopState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_boot_status(self):
        """
        Output:
            ethercat_used -> bool
            safety_used -> bool
            safety_mcu -> bool
            safety_connected -> bool
            main_pw_relay_on -> bool
            safety_pw_relay_on -> bool
            robot_pw_supply_on -> bool
            ethercat_connected -> bool
            control_on -> bool
        """
        response = self.boot.GetBootStatus(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # IO board and Endtool port interfaces
    ############################
    def get_di(self):
        """
        Output:
            signals -> DigitalSignal[] {address: uint32, state: DigitalState}
        """
        response = self.device.GetDI(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_do(self):
        """
        Output:
            signals -> DigitalSignal[] {address: uint32, state: DigitalState}
        """
        response = self.device.GetDO(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_do(self, do_signal_list: list):
        """
        Input:
            do_signal_list -> [(address, True/False), ...] or [{'address': int, 'state': bool}, ...]
        Output:
            response -> {code, msg}
        """
        # Normalize inputs: [(addr, state)] or [{'address':..,'state':..}] -> DigitalSignal
        norm_list = []
        for item in (do_signal_list or []):
            if isinstance(item, (tuple, list)) and len(item) == 2:
                addr, state = item
                norm_list.append({'address': addr, 'state': state})
            else:
                norm_list.append(item)
        response = self.device.SetDO(device_msgs.DigitalList(
            signals=self.__to_digital_request_list__(norm_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_di(self, di_signal_list: list):
        """
        Input:
            di_signal_list -> [(address, True/False), ...] or [{'address': int, 'state': bool}, ...]
        Output:
            response -> {code, msg}
        """
        norm_list = []
        for item in (di_signal_list or []):
            if isinstance(item, (tuple, list)) and len(item) == 2:
                addr, state = item
                norm_list.append({'address': addr, 'state': state})
            else:
                norm_list.append(item)
        response = self.device.SetDI(device_msgs.DigitalList(
            signals=self.__to_digital_request_list__(norm_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def sim_di_config(self, di_signals: dict):
        """
        Input:
            di_signals -> dict {sim_mode: DISimMode, pulse_period_ms: uint32, signals: DigitalSignal[]}
        Output:
            response -> {code, msg}
        """
        req = device_msgs.DISignals()
        ParseDict(di_signals, req)
        response = self.device.SimDIConfig(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ai(self) -> list:
        """
        Output:
            signals -> AnalogSignal[] {address: uint32, voltage: int32}
        """
        response = self.device.GetAI(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_ai(self, ai_signal_list: list):
        """
        Input:
            ai_signal_list -> [(address, voltage), ...] or [{'address': int, 'voltage': int}, ...]
        Output:
            response -> {code, msg}
        """
        response = self.device.SetAI(device_msgs.AnalogList(
            signals=self.__to_analog_request_list__(ai_signal_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ao(self) -> list:
        """
        Output:
            signals -> AnalogSignal[] {address: uint32, voltage: int32}
        """
        response = self.device.GetAO(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_ao(self, ao_signal_list: list):
        """
        Input:
            ao_signal_list -> [(address, voltage), ...] or [{'address': int, 'voltage': int}, ...]
        Output:
            response -> {code, msg}
        """
        response = self.device.SetAO(device_msgs.AnalogList(
            signals=self.__to_analog_request_list__(ao_signal_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_di(self) -> list:
        """
        Output:
            signals -> EndtoolSignal[] {port: string, states: EndtoolState[]}
        """
        response = self.device.GetEndDI(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_endtool_di(self, end_di_signal_list: list):
        """
        Input:
            end_di_signal_list -> [(port, [states]), ...] or [{'port': str, 'states': list}, ...]
        Output:
            response -> {code, msg}
        """
        response = self.device.SetEndDI(device_msgs.EndtoolSignalList(
            signals=self.__to_endtool_signal_list__(end_di_signal_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_do(self) -> list:
        """
        Output:
            signals -> EndtoolSignal[] {port: string, states: EndtoolState[]}
        """
        response = self.device.GetEndDO(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_endtool_do(self, end_do_signal_list: list):
        """
        Input:
            end_do_signal_list -> [(port, [states]), ...] or [{'port': str, 'states': list}, ...]
        Output:
            response -> {code, msg}
        """
        response = self.device.SetEndDO(device_msgs.EndtoolSignalList(
            signals=self.__to_endtool_signal_list__(end_do_signal_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_ai(self) -> list:
        """
        Output:
            signals -> AnalogSignal[] {address: uint32, voltage: int32}
        """
        response = self.device.GetEndAI(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_endtool_ai(self, end_ai_signal_list: list):
        """
        Input:
            end_ai_signal_list -> [(address, voltage), ...] or [{'address': int, 'voltage': int}, ...]
        Output:
            response -> {code, msg}
        """
        response = self.device.SetEndAI(device_msgs.AnalogList(
            signals=self.__to_analog_request_list__(end_ai_signal_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_ao(self) -> list:
        """
        Output:
            signals -> AnalogSignal[] {address: uint32, voltage: int32}
        """
        response = self.device.GetEndAO(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_endtool_ao(self, end_ao_signal_list: list):
        """
        Input:
            end_ao_signal_list -> [(address, voltage), ...] or [{'address': int, 'voltage': int}, ...]
        Output:
            response -> {code, msg}
        """
        response = self.device.SetEndAO(device_msgs.AnalogList(
            signals=self.__to_analog_request_list__(end_ao_signal_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_rs485_rx(self) -> dict:
        """
        Output:
            word1 -> uint32
            word2 -> uint32
            word3 -> uint32
            word4 -> uint32
            word5 -> uint32
            control -> uint32
            num -> uint32
        """
        response = self.device.GetEndRS485Rx(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_rs485_tx(self) -> dict:
        """
        Output:
            word1 -> uint32
            word2 -> uint32
            word3 -> uint32
            word4 -> uint32
            word5 -> uint32
            status -> uint32
            num -> uint32
        """
        response = self.device.GetEndRS485Tx(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_endtool_rs485_rx(self, word1: int, word2: int):
        """
        Input:
            word1 -> uint32
            word2 -> uint32
        Output:
            response -> {code, msg}
        """
        response = self.device.SetEndRS485Rx(common_msgs.EndtoolRS485Rx(
            word1=word1, word2=word2
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_endtool_led_dim(self, led_dim):
        """
        Input:
            led_dim -> uint32
        """
        response = self.device.SetEndLedDim(device_msgs.EndLedDim(led_dim=led_dim))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    
    def execute_tool(self, name: str):
        """
        Input:
            name -> string (tool name)
        """
        response = self.device.ExecuteTool(common_msgs.Name(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_el5001(self):
        # Not defined in proto_ori Device; 
        return {"error": "Unsupported RPC on Device: GetEL5001 (not in proto_ori)"}

    def get_el5101(self):
        # Not defined in proto_ori Device;
        return {"error": "Unsupported RPC on Device: GetEL5101 (not in proto_ori)"}

    def get_brake_control_style(self):
        """
        Output:
            style -> ControlStyle (UNAVAILABLE=0, CONCURRENT=1, INDIVIDUAL=2)
            response -> {code, msg}
        """
        response = self.device.GetBrakeControlStyle(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_device_info(self):
        """
        Output:
            num_joints -> uint32
            robot_serial -> string
            payload -> float
            robot_dof -> uint32
            robot_name -> string
            cb_serial -> string
            io_board_fw_ver -> string
            core_board_fw_vers -> string[]
            endtool_board_fw_ver -> string
            controller_ver -> string
            controller_detail -> string
            controller_date -> string
            controller_type -> string
            controller_platform -> string
            teleop_loaded -> bool
            calibrated -> bool
            use_safety_io -> bool
            use_npad -> bool
            use_indykey -> bool
            use_auto_mode -> bool
            use_safety_mcu -> bool
            response -> {code, msg}
        """
        response = self.device.GetDeviceInfo(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_conveyor(self):
        """
        Output:
            name -> string
            encoder -> {type, channel1, channel2, sample_num, mm_per_tick, vel_const_mmps, reversed}
            trigger -> {type, channel, detect_rise}
            offset_dist -> float
            working_dist -> float
            direction -> Vector
            starting_pose -> {q: float[], p: float[]}
            terminal_pose -> {q: float[], p: float[]}
            tool_link -> int32
        """
        response = self.device.GetConveyor(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_name(self, name: str):
        """
        Input:
            name -> string
        Output:
            response -> {code, msg}
        """
        response = self.device.SetConveyorName(common_msgs.Name(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_by_name(self, name: str):
        """
        Input:
            name -> string
        Output:
            response -> {code, msg}
        """
        response = self.device.SetConveyorByName(common_msgs.Name(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_encoder(self, encoder_type, channel1: int, channel2: int, sample_num: int,
                           mm_per_tick: float, vel_const_mmps: float, reversed: bool):
        """
        Input:
            encoder_type -> EncoderType (CONSTANT=0, QUADRATURE=1, RISING=2, FALLING=3, MODBUS=4)
            channel1 -> int64
            channel2 -> int64
            sample_num -> int64
            mm_per_tick -> float
            vel_const_mmps -> float
            reversed -> bool
        Output:
            response -> {code, msg}
        """
        response = self.device.SetConveyorEncoder(
            device_msgs.Encoder(type=encoder_type,
                                channel1=channel1, channel2=channel2, sample_num=sample_num,
                                mm_per_tick=mm_per_tick, vel_const_mmps=vel_const_mmps,
                                reversed=reversed)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_trigger(self, trigger_type, channel: int, detect_rise: bool):
        """
        Input:
            trigger_type -> TriggerType (DIGITAL=0, MODBUS=1)
            channel -> int64
            detect_rise -> bool
        Output:
            response -> {code, msg}
        """
        response = self.device.SetConveyorTrigger(
            device_msgs.Trigger(type=trigger_type, channel=channel, detect_rise=detect_rise)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_offset(self, offset_mm):
        """
        Input:
            offset_mm -> float
        Output:
            response -> {code, msg}
        """
        response = self.device.SetConveyorOffset(common_msgs.Float(value=offset_mm))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_locked_joint(self, index: int):
        """
        Input:
            index -> int
        Output:
            response -> {code, msg}
        """
        response = self.device.SetConveyorLockedJoint(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_tool_link(self, index: int):
        """
        Input:
            index -> int
        Output:
            response -> {code, msg}
        """
        response = self.device.SetConveyorToolLink(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_starting_pose(self, jpos, tpos):
        """
        Input:
            jpos -> float[] (joint positions)
            tpos -> float[] (task positions)
        Output:
            response -> {code, msg}
        """
        response = self.device.SetConveyorStartingPose(
            common_msgs.PosePair(q=jpos, p=tpos)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_terminal_pose(self, jpos, tpos):
        """
        Input:
            jpos -> float[] (joint positions)
            tpos -> float[] (task positions)
        Output:
            response -> {code, msg}
        """
        response = self.device.SetConveyorTerminalPose(
            common_msgs.PosePair(q=jpos, p=tpos)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_conveyor_state(self):
        """
        Output:
            velocity -> float
            triggered -> float
        """
        response = self.device.GetConveyorState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_conveyor_object_distances(self):
        """
        Output:
            dists -> float[] (mm, latest on the last index)
        """
        response = self.device.GetConveyorObjectDistances(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_sander_command(self, sander_type, ip: str, speed: float, state: bool):
        """
        Input:
            sander_type -> SanderType (SANDER_ONROBOT=0)
            ip -> string
            speed -> float
            state -> bool
        Output:
            response -> {code, msg}
        """
        response = self.device.SetSanderCommand(
            device_msgs.SanderCommand(type=sander_type, ip=ip, speed=speed, state=state))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_sander_command(self):
        """
        Output:
            type -> SanderType
            ip -> string
            speed -> float
            state -> bool
        """
        response = self.device.GetSanderCommand(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def add_photoneo_calib_point(self, vision_name, px, py, pz):
        """
        Input:
            vision_name -> string
            px -> double
            py -> double
            pz -> double
        Output:
            response -> {code, msg}
        """
        response = self.device.AddPhotoneoCalibPoint(
            device_msgs.AddPhotoneoCalibPointReq(vision_name=vision_name, px=px, py=py, pz=pz))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_photoneo_detection(self, vision_server, object, frame_type):
        """
        Input:
            vision_server -> VisionServer {name, vision_server_type, ip, port}
            object -> string
            frame_type -> VisionFrameType (OBJECT=0, END_EFFECTOR=1)
        Output:
            frame -> float[]
            frame_type -> VisionFrameType
            object -> string
            detected -> bool
            passed -> bool
            msg -> string
        """
        response = self.device.GetPhotoneoDetection(
            device_msgs.VisionRequest(vision_server=vision_server, object=object, frame_type=frame_type))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_photoneo_retrieval(self, vision_server, object, frame_type):
        """
        Input:
            vision_server -> VisionServer {name, vision_server_type, ip, port}
            object -> string
            frame_type -> VisionFrameType (OBJECT=0, END_EFFECTOR=1)
        Output:
            frame -> float[]
            frame_type -> VisionFrameType
            object -> string
            detected -> bool
            passed -> bool
            msg -> string
        """
        response = self.device.GetPhotoneoRetrieval(
            device_msgs.VisionRequest(vision_server=vision_server, object=object, frame_type=frame_type))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def configure_pickit3d(self, config: dict):
        """
        Input:
            config -> dict {vision_server: {name, vision_server_type, ip, port}, setup_id: uint32, product_id: uint32}
        Output:
            response -> {code, msg}
        """
        req = device_msgs.ConfigurePickit3DReq()
        ParseDict(config, req)
        response = self.device.ConfigurePickit3D(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_pickit3d_detection(self, request: dict):
        """
        Input:
            request -> dict {vision_server, object, frame_type, solution_id, vision_id}
        Output:
            frame -> float[]
            frame_type -> VisionFrameType
            object -> string
            detected -> bool
            passed -> bool
            msg -> string
        """
        req = device_msgs.VisionRequest()
        ParseDict(request, req)
        response = self.device.GetPickit3DDetection(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_pickit3d_retrieval(self, request: dict):
        """
        Input:
            request -> dict {vision_server, object, frame_type, solution_id, vision_id}
        Output:
            frame -> float[]
            frame_type -> VisionFrameType
            object -> string
            detected -> bool
            passed -> bool
            msg -> string
        """
        req = device_msgs.VisionRequest()
        ParseDict(request, req)
        response = self.device.GetPickit3DRetrieval(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ft_sensor_data(self):
        """
        Output:
            ft_Fx -> float
            ft_Fy -> float
            ft_Fz -> float
            ft_Tx -> float
            ft_Ty -> float
            ft_Tz -> float
            response -> {code, msg}
        """
        response = self.device.GetFTSensorData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_load_factors(self):
        """
        Output:
            percents -> int32[] (percent)
            torques -> float[] (Nm)
            response -> {code, msg}
        """
        response = self.device.GetLoadFactors(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_auto_mode(self, on: bool):
        """
        Input:
            on -> bool
        Output:
            msg -> string
        """
        response = self.device.SetAutoMode(device_msgs.SetAutoModeReq(on=on))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def check_auto_mode(self):
        """
        Output:
            on -> bool
            msg -> string
        """
        response = self.device.CheckAutoMode(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def check_reduced_mode(self):
        """
        Output:
            on -> bool
            msg -> string
        """
        response = self.device.CheckReducedMode(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_safety_function_state(self):
        """
        Output:
            id -> uint32
            state -> uint32
            response -> {code, msg}
        """
        response = self.device.GetSafetyFunctionState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def request_safety_function(self, id, state):
        """
        Input:
            id -> uint32
            state -> uint32
        Output:
            response -> {code, msg}
        """
        response = self.device.RequestSafetyFunction(
            device_msgs.SafetyFunctionState(id = id, state = state))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_safety_control_data(self):
        """
        Output:
            auto_mode -> bool
            reduced_mode -> bool
            enabler_pressed -> bool
            safety_state -> {id, state, response}
        """
        response = self.device.GetSafetyControlData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_gripper_data(self) -> list:
        """
        Output:
            gripper_type -> GripperType (NONE=0, ROBOTIQ_GRIPPER=1, DH_GRIPPER=2, APICOO_SUSGRIP=3)
            gripper_position -> int32
            gripper_state -> int32
        """
        response = self.device.GetGripperData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_gripper_command(self, command, gripper_type, pvt_data):
        """
        Input:
            command -> GripperCommandType (AUTO_DETECT=0, ACTIVATE=1, RE_ACTIVATE=2, SET_PVT=3)
            gripper_type -> GripperType (NONE=0, ROBOTIQ_GRIPPER=1, DH_GRIPPER=2, APICOO_SUSGRIP=3)
            pvt_data -> int32[]
        """
        response = self.device.SetGripperCommand(device_msgs.GripperCommand(gripper_command=command,
                                                                            gripper_type=gripper_type,
                                                                            gripper_pvt_data=pvt_data))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_brakes(self, brake_state_list: list):
        """
        Input:
            brake_state_list -> bool[] (per-joint brake states)
        Output:
            response -> {code, msg}
        """
        motor_list = []
        motor_idx = 0
        for brake_state in brake_state_list:
            motor_list.append(device_msgs.Motor(index=motor_idx, enable=brake_state))
            motor_idx += 1

        response = self.device.SetBrakes(device_msgs.MotorList(
            motors=list(motor_list)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    def set_servo_all(self, enable=True):
        """
        Input:
            enable -> bool
        Output:
            response -> {code, msg}
        """
        response = self.device.SetServoAll(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_servo(self, index, enable=True):
        """
        Input:
            index -> uint32
            enable -> bool
        Output:
            response -> {code, msg}
        """
        response = self.device.SetServo(device_msgs.Servo(index=index, enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
        
    ############################
    # CRI Funtions (CRI)
    ############################
    def activate_cri(self, on: bool) -> dict:
        """
        Input:
            on -> bool
        Output:
            response -> {code, msg}
        """
        response = self.cri.ActiveCRIVel(common_msgs.State(enable=on))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def is_cri_active(self) -> dict:
        """
        Output:
            enable -> bool
        """
        response = self.cri.IsSFDLogin(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def login_cri_server(self, email: str, token: str) -> dict:
        """
        Input:
            email -> string
            token -> string
        Output:
            response -> {code, msg}
        """
        response = self.cri.LoginSFD(cri_msgs.SFDAccount(email=email, token=token))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def is_cri_login(self) -> dict:
        """
        Output:
            enable -> bool
        """
        response = self.cri.IsSFDLogin(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_cri_target(self, pn: str, fn: str, rn: str) -> dict:
        """
        Input:
            pn -> string
            fn -> string
            rn -> string
        Output:
            response -> {code, msg}
        """
        response = self.cri.SelectSFDTarget(cri_msgs.SFDTarget(pn=pn, fn=fn, rn=rn))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_cri_option(self, on: bool) -> dict:
        """
        Input:
            on -> bool
        Output:
            response -> {code, msg}
        """
        response = self.cri.ActiveCRIVel(common_msgs.State(enable=on))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_cri_proj_list(self) -> dict:
        """
        Output:
            list -> string
        """
        response = self.cri.GetSFDProjList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_cri(self) -> dict:
        """
        Output:
            time -> double
            cri -> double
            velRatio -> double
        """
        response = self.cri.GetCRI(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def logout_cri_server(self) -> dict:
        """
        Output:
            response -> {code, msg}
        """
        response = self.cri.LogoutSFD(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def generate_cri_token(self, email: str, token: str) -> dict:
        """
        Input:
            email -> string
            token -> string
        Output:
            response -> {code, msg}
        """
        response = self.cri.GenerateSFDToken(cri_msgs.SFDAccount(email=email, token=token))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def save_cri_login_info(self, email: str, token: str) -> dict:
        """
        Input:
            email -> string
            token -> string
        Output:
            response -> {code, msg}
        """
        response = self.cri.SaveSFDLoginInfo(cri_msgs.SFDAccount(email=email, token=token))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def load_cri_login_info(self) -> dict:
        """
        Output:
            email -> string
            token -> string
        """
        response = self.cri.LoadSFDLoginInfo(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_cri_login_info(self) -> dict:
        """
        Output:
            email -> string
            token -> string
        """
        response = self.cri.GetSFDLoginInfo(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def is_cri_target_valid(self) -> dict:
        """
        Output:
            enable -> bool
        """
        response = self.cri.IsSFDTargetValid(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def release_cri_target(self) -> dict:
        """
        Output:
            enable -> bool
        """
        response = self.cri.ReleaseSFDTarget(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_cri_target(self) -> dict:
        """
        Output:
            pn -> string
            fn -> string
            rn -> string
            iso -> bool
        """
        response = self.cri.GetSFDTarget(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def save_cri_auto_set(self, auto_set: dict) -> dict:
        """
        Input:
            auto_set -> dict {login: bool, pn: string, fn: string, rn: string, iso: bool}
        Output:
            response -> {code, msg}
        """
        req = cri_msgs.SFDAutoSet()
        ParseDict(auto_set, req)
        response = self.cri.SaveSFDAutoSet(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def load_cri_auto_set(self) -> dict:
        """
        Output:
            login -> bool
            pn -> string
            fn -> string
            rn -> string
            iso -> bool
        """
        response = self.cri.LoadSFDAutoSet(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Motion Control (Move commands)
    ############################
    def stop_motion(self, stop_category=StopCategory.CAT2) -> dict:
        """
        Input:
            stop_category -> StopCategory (CAT0=0, CAT1=1, CAT2=2)
        Output:
            response -> {code, msg}
        """
        response = self.control.StopMotion(common_msgs.StopCat(category=stop_category))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movej(self, jtarget,
              blending_type=BlendingType.NONE,
              base_type=JointBaseType.ABSOLUTE,
              blending_radius=0.0,
              vel_ratio=Limits.JogVelRatioDefault,
              acc_ratio=Limits.JogAccRatioDefault,
              post_condition=PostCondition(),
              teaching_mode=False) -> dict:
        """
        Input:
            jtarget -> float[] (joint angles in deg)
            blending_type -> BlendingType (NONE=0, OVERRIDE=1, DUPLICATE=2)
            base_type -> JointBaseType (ABSOLUTE=0, RELATIVE=1)
            blending_radius -> float
            vel_ratio -> float (0~100)
            acc_ratio -> float (0~100)
            post_condition -> PostCondition
            teaching_mode -> bool
        Output:
            response -> {code, msg}
        """
        if teaching_mode and base_type!=JointBaseType.ABSOLUTE:
            if self.get_robot_data()['op_state'] == 6:
                print("Robot is moving. Cannot execute movej with teaching_mode=True and base_type!=ABSOLUTE.")
                return {"error": "Robot is in motion, command aborted."}
            
        jtarget = control_msgs.TargetJ(j_start=[], j_target=list(jtarget), base_type=base_type)
        blending = control_msgs.BlendingType(type=blending_type, blending_radius=blending_radius)
        post_cond = control_msgs.MotionCondition()
        if post_condition is not None:
            post_cond = control_msgs.MotionCondition(
                type_cond=post_condition.condition_type,
                type_react=post_condition.reaction_type,
                const_cond=post_condition.const_cond,
                io_cond=control_msgs.IOCondition(
                    di=self.__to_digital_request_list__(
                        [{'address': di[0], 'state': di[1]} for di in post_condition.digital_inputs]),
                    # di=self.__to_digital_request_list__(post_condition.digital_inputs),
                    # end_di=self.__to_digital_request_list__(post_condition['enddi_condition']),
                ),
            )

        response = self.control.MoveJ(control_msgs.MoveJReq(
            target=jtarget,
            blending=blending,
            vel_ratio=vel_ratio, acc_ratio=acc_ratio,
            post_condition=post_cond,
            teaching_mode=teaching_mode
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movej_time(self, jtarget,
                   blending_type=BlendingType.NONE,
                   base_type=JointBaseType.ABSOLUTE,
                   blending_radius=0.0,
                   move_time=5.0,
                   post_condition=PostCondition()) -> dict:
        """
        Input:
            jtarget -> float[] (joint angles in deg)
            blending_type -> BlendingType (NONE=0, OVERRIDE=1, DUPLICATE=2)
            base_type -> JointBaseType (ABSOLUTE=0, RELATIVE=1)
            blending_radius -> float
            move_time -> float (seconds)
            post_condition -> PostCondition
        Output:
            response -> {code, msg}
        """
        jtarget = control_msgs.TargetJ(j_start=[], j_target=list(jtarget), base_type=base_type)
        blending = control_msgs.BlendingType(type=blending_type, blending_radius=blending_radius)
        post_cond = control_msgs.MotionCondition()
        if post_condition is not None:
            post_cond = control_msgs.MotionCondition(
                type_cond=post_condition.condition_type,
                type_react=post_condition.reaction_type,
                const_cond=post_condition.const_cond,
                io_cond=control_msgs.IOCondition(
                    di=self.__to_digital_request_list__(
                        [{'address': di[0], 'state': di[1]} for di in post_condition.digital_inputs]),
                    # di=self.__to_digital_request_list__(post_condition.digital_inputs),
                    # end_di=self.__to_digital_request_list__(post_condition['enddi_condition']),
                ),
            )

        response = self.control.MoveJT(control_msgs.MoveJTReq(
            target=jtarget,
            blending=blending,
            time=move_time,
            post_condition=post_cond
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    
    def movel(self, ttarget,
              blending_type=BlendingType.NONE,
              base_type=TaskBaseType.ABSOLUTE,
              blending_radius=0.0,
              vel_ratio=Limits.JogVelRatioDefault,
              acc_ratio=Limits.JogAccRatioDefault,
              post_condition=PostCondition(),
              teaching_mode=False,
              bypass_singular=False) -> dict:
        """
        Input:
            ttarget -> float[6] [mm, mm, mm, deg, deg, deg]
            blending_type -> BlendingType (NONE=0, OVERRIDE=1, DUPLICATE=2)
            base_type -> TaskBaseType (ABSOLUTE=0, RELATIVE=1, TCP=2)
            blending_radius -> float
            vel_ratio -> float (0~100)
            acc_ratio -> float (0~100)
            post_condition -> PostCondition
            teaching_mode -> bool
            bypass_singular -> bool
        Output:
            response -> {code, msg}
        """
        if teaching_mode and base_type!=TaskBaseType.ABSOLUTE:
            if self.get_robot_data()['op_state'] == 6:
                print("Robot is moving. Cannot execute movel with teaching_mode=True and base_type!=ABSOLUTE.")
                return {"error": "Robot is in motion, command aborted."}
        
        ptarget = control_msgs.TargetP(t_start=[], t_target=list(ttarget), base_type=base_type)
        blending = control_msgs.BlendingType(type=blending_type, blending_radius=blending_radius)
        post_cond = control_msgs.MotionCondition()
        if post_condition is not None:
            post_cond = control_msgs.MotionCondition(
                type_cond=post_condition.condition_type,
                type_react=post_condition.reaction_type,
                const_cond=post_condition.const_cond,
                io_cond=control_msgs.IOCondition(
                    di=self.__to_digital_request_list__(
                        [{'address': di[0], 'state': di[1]} for di in post_condition.digital_inputs]),
                    # di=self.__to_digital_request_list__(post_condition.digital_inputs),
                    # end_di=self.__to_digital_request_list__(post_condition['enddi_condition']),
                ),
            )

        response = self.control.MoveL(control_msgs.MoveLReq(
            target=ptarget,
            blending=blending,
            vel_ratio=vel_ratio, acc_ratio=acc_ratio,
            post_condition=post_cond,
            teaching_mode=teaching_mode,
            bypass_singular=bypass_singular
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movel_time(self, ttarget,
                   blending_type=BlendingType.NONE,
                   base_type=TaskBaseType.ABSOLUTE,
                   blending_radius=0.0,
                   move_time=5.0,
                   post_condition=PostCondition()) -> dict:
        """
        Input:
            ttarget -> float[6] [mm, mm, mm, deg, deg, deg]
            blending_type -> BlendingType (NONE=0, OVERRIDE=1, DUPLICATE=2)
            base_type -> TaskBaseType (ABSOLUTE=0, RELATIVE=1, TCP=2)
            blending_radius -> float
            move_time -> float (seconds)
            post_condition -> PostCondition
        Output:
            response -> {code, msg}
        """
        ptarget = control_msgs.TargetP(t_start=[], t_target=list(ttarget), base_type=base_type)
        blending = control_msgs.BlendingType(type=blending_type, blending_radius=blending_radius)
        post_cond = control_msgs.MotionCondition()
        if post_condition is not None:
            post_cond = control_msgs.MotionCondition(
                type_cond=post_condition.condition_type,
                type_react=post_condition.reaction_type,
                const_cond=post_condition.const_cond,
                io_cond=control_msgs.IOCondition(
                    di=self.__to_digital_request_list__(
                        [{'address': di[0], 'state': di[1]} for di in post_condition.digital_inputs]),
                    # di=self.__to_digital_request_list__(post_condition.digital_inputs),
                    # end_di=self.__to_digital_request_list__(post_condition['enddi_condition']),
                ),
            )

        response = self.control.MoveLT(control_msgs.MoveLTReq(
            target=ptarget,
            blending=blending,
            time=move_time,
            post_condition=post_cond
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movelf(self, ttarget, enabledaxis, desforce,
               blending_type=BlendingType.NONE,
               base_type=TaskBaseType.ABSOLUTE,
               blending_radius=0.0,
               vel_ratio=Limits.JogVelRatioDefault,
               acc_ratio=Limits.JogAccRatioDefault,
               post_condition=PostCondition(),
               teaching_mode=False) -> dict:
        """
        Input:
            ttarget -> float[6] [mm, mm, mm, deg, deg, deg]
            enabledaxis -> bool[] (force axes enabled)
            desforce -> float[] (desired force per axis)
            blending_type -> BlendingType (NONE=0, OVERRIDE=1, DUPLICATE=2)
            base_type -> TaskBaseType (ABSOLUTE=0, RELATIVE=1, TCP=2)
            blending_radius -> float
            vel_ratio -> float (0~100)
            acc_ratio -> float (0~100)
            post_condition -> PostCondition
            teaching_mode -> bool
        Output:
            response -> {code, msg}
        """
        ptarget = control_msgs.TargetP(t_start=[], t_target=list(ttarget), base_type=base_type)
        blending = control_msgs.BlendingType(type=blending_type, blending_radius=blending_radius)
        post_cond = control_msgs.MotionCondition()
        if post_condition is not None:
            post_cond = control_msgs.MotionCondition(
                type_cond=post_condition.condition_type,
                type_react=post_condition.reaction_type,
                const_cond=post_condition.const_cond,
                io_cond=control_msgs.IOCondition(
                    di=self.__to_digital_request_list__(
                        [{'address': di[0], 'state': di[1]} for di in post_condition.digital_inputs]),
                    # di=self.__to_digital_request_list__(post_condition.digital_inputs),
                    # end_di=self.__to_digital_request_list__(post_condition['enddi_condition']),
                ),
            )

        response = self.control.MoveLF(control_msgs.MoveLFReq(
            target=ptarget,
            blending=blending,
            vel_ratio=vel_ratio, acc_ratio=acc_ratio,
            des_force=desforce, enabled_force=enabledaxis,
            post_condition=post_cond,
            teaching_mode=teaching_mode
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_transformed_ft_sensor_data(self):
        """
        Output:
            ft_Fx -> float (N)
            ft_Fy -> float (N)
            ft_Fz -> float (N)
            ft_Tx -> float (N*m)
            ft_Ty -> float (N*m)
            ft_Tz -> float (N*m)
            response -> {code, msg}
        """
        response = self.control.GetTransformedFTSensorData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movec(self, tpos0, tpos1,
              blending_type=BlendingType.NONE,
              base_type=TaskBaseType.ABSOLUTE,
              angle=0.0,
              setting_type=CircularSettingType.POINT_SET,
              move_type=control_msgs.CONSTANT,
              blending_radius=0.0,
              vel_ratio=Limits.JogVelRatioDefault,
              acc_ratio=Limits.JogAccRatioDefault,
              post_condition=PostCondition(),
              teaching_mode=False,
              bypass_singular=False) -> dict:
        """
        Input:
            tpos0 -> float[6] [mm, mm, mm, deg, deg, deg] (via point)
            tpos1 -> float[6] [mm, mm, mm, deg, deg, deg] (target point)
            blending_type -> BlendingType (NONE=0, OVERRIDE=1, DUPLICATE=2)
            base_type -> TaskBaseType (ABSOLUTE=0, RELATIVE=1, TCP=2)
            angle -> float (degrees)
            setting_type -> CircularSettingType (POINT_SET=0, CENTER_AXIS=1)
            move_type -> CircularMovingType (CONSTANT=0, RADIAL=1, SMOOTH=2)
            blending_radius -> float
            vel_ratio -> float (0~100)
            acc_ratio -> float (0~100)
            post_condition -> PostCondition
            teaching_mode -> bool
            bypass_singular -> bool
        Output:
            response -> {code, msg}
        """

        ctarget = control_msgs.TargetC(t_start=[], t_pos0=list(tpos0), t_pos1=list(tpos1),
                                       base_type=base_type)
        blending = control_msgs.BlendingType(type=blending_type, blending_radius=blending_radius)
        post_cond = control_msgs.MotionCondition()
        if post_condition is not None:
            post_cond = control_msgs.MotionCondition(
                type_cond=post_condition.condition_type,
                type_react=post_condition.reaction_type,
                const_cond=post_condition.const_cond,
                io_cond=control_msgs.IOCondition(
                    di=self.__to_digital_request_list__(
                        [{'address': di[0], 'state': di[1]} for di in post_condition.digital_inputs]),
                    # di=self.__to_digital_request_list__(post_condition.digital_inputs),
                    # end_di=self.__to_digital_request_list__(post_condition['enddi_condition']),
                ),
            )

        response = self.control.MoveC(control_msgs.MoveCReq(
            target=ctarget,
            blending=blending,
            angle=angle,
            setting_type=setting_type,
            move_type=move_type,
            vel_ratio=vel_ratio, acc_ratio=acc_ratio,
            post_condition=post_cond,
            teaching_mode=teaching_mode,
            bypass_singular=bypass_singular
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movec_time(self, tpos0, tpos1,
               blending_type=BlendingType.NONE,
               base_type=TaskBaseType.ABSOLUTE,
               angle=90.0,
               setting_type=CircularSettingType.POINT_SET,
               move_type=control_msgs.CONSTANT,
               blending_radius=0.0,
               move_time=5.0,
               post_condition=PostCondition()) -> dict:
        """
        Input:
            tpos0 -> float[6] [mm, mm, mm, deg, deg, deg] (via point)
            tpos1 -> float[6] [mm, mm, mm, deg, deg, deg] (target point)
            blending_type -> BlendingType (NONE=0, OVERRIDE=1, DUPLICATE=2)
            base_type -> TaskBaseType (ABSOLUTE=0, RELATIVE=1, TCP=2)
            angle -> float (degrees)
            setting_type -> CircularSettingType (POINT_SET=0, CENTER_AXIS=1)
            move_type -> CircularMovingType (CONSTANT=0, RADIAL=1, SMOOTH=2)
            blending_radius -> float
            move_time -> float (seconds)
            post_condition -> PostCondition
        Output:
            response -> {code, msg}
        """
        
        ctarget = control_msgs.TargetC(t_start=[], t_pos0=list(tpos0), t_pos1=list(tpos1),
                                       base_type=base_type)
        
        blending = control_msgs.BlendingType(type=blending_type, blending_radius=blending_radius)
        post_cond = control_msgs.MotionCondition()
        if post_condition is not None:
            post_cond = control_msgs.MotionCondition(
                type_cond=post_condition.condition_type,
                type_react=post_condition.reaction_type,
                const_cond=post_condition.const_cond,
                io_cond=control_msgs.IOCondition(
                    di=self.__to_digital_request_list__(
                        [{'address': di[0], 'state': di[1]} for di in post_condition.digital_inputs]),
                    # di=self.__to_digital_request_list__(post_condition.digital_inputs),
                    # end_di=self.__to_digital_request_list__(post_condition['enddi_condition']),
                ),
            )

        response = self.control.MoveCT(control_msgs.MoveCTReq(
            target=ctarget,
            blending=blending,
            angle=angle,
            setting_type=setting_type,
            move_type=move_type,
            time=move_time,
            post_condition=post_cond
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def move_gcode(self, gcode_file,
                   is_smooth_mode=False,
                   smooth_radius=0.0,
                   vel_ratio=Limits.JogVelRatioDefault,
                   acc_ratio=Limits.JogAccRatioDefault) -> dict:
        """
        Input:
            gcode_file -> string
            is_smooth_mode -> bool
            smooth_radius -> float (mm)
            vel_ratio -> float (percent)
            acc_ratio -> float (percent)
        Output:
            response -> {code, msg}
        """
        
        gcode_req = control_msgs.MoveGcodeReq(gcode_file=gcode_file,
                                              is_smooth_mode=is_smooth_mode,
                                              smooth_radius=smooth_radius,
                                              vel_ratio=vel_ratio,
                                              acc_ratio=acc_ratio)
        
        response = self.control.MoveGcode(gcode_req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Move Trajectory
    ############################

    ##
    # @brief move along joint trajectory
    # @remark all arguments are NxD arrays (N: number of points, D: DOF)
    # @param q_list joint values (unit: rads)
    # @param qdot_list joint velocities (unit: rads/s)
    # @param qddot_list joint accelerations (unit: rads/s^2)
    def move_joint_traj(self, q_list: List[List[float]], qdot_list: List[List[float]],
                        qddot_list: List[List[float]]) -> dict:
        """
        Input:
            q_list -> float[][] (NxDOF, joint positions in rad)
            qdot_list -> float[][] (NxDOF, joint velocities in rad/s)
            qddot_list -> float[][] (NxDOF, joint accelerations in rad/s^2)
        Output:
            response -> {code, msg}
        """
        traj_req = control_msgs.MoveJointTrajReq(q_list=list(map(lambda x: common_msgs.Vector(values=x), q_list)),
                                                 qdot_list=list(map(lambda x: common_msgs.Vector(values=x), qdot_list)),
                                                 qddot_list=list(
                                                     map(lambda x: common_msgs.Vector(values=x), qddot_list)))
        response = self.control.MoveJointTraj(traj_req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ##
    # @brief move along joint trajectory
    # @remark all arguments are Nx6 arrays (N: number of points)
    # @param p_list task positions (xyzuvw), unit: m & rads
    # @param pdot_list task velocities (v, w), unit: m/s & rads/s
    # @param pddot_list task accelerations (v, w), unit: m/s^2 & rads/s^2
    def move_task_traj(self, p_list: List[List[float]], pdot_list: List[List[float]],
                       pddot_list: List[List[float]]) -> dict:
        """
        Input:
            p_list -> float[][] (Nx6, task positions xyzuvw in m & rad)
            pdot_list -> float[][] (Nx6, task velocities in m/s & rad/s)
            pddot_list -> float[][] (Nx6, task accelerations in m/s^2 & rad/s^2)
        Output:
            response -> {code, msg}
        """
        traj_req = control_msgs.MoveTaskTrajReq(p_list=list(map(lambda x: common_msgs.Vector(values=x), p_list)),
                                                pdot_list=list(map(lambda x: common_msgs.Vector(values=x), pdot_list)),
                                                pddot_list=list(
                                                    map(lambda x: common_msgs.Vector(values=x), pddot_list)))
        response = self.control.MoveTaskTraj(traj_req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def move_conveyor(self,
                     post_condition=PostCondition(),
                     teaching_mode=False, bypass_singular=False,
                     acc_ratio=Limits.JogAccRatioDefault) -> dict:
        """
        Input:
            post_condition -> PostCondition
            teaching_mode -> bool
            bypass_singular -> bool
            acc_ratio -> float (0~1000)
        Output:
            response -> {code, msg}
        """
        post_cond = control_msgs.MotionCondition()
        if post_condition is not None:
            post_cond = control_msgs.MotionCondition(
                type_cond=post_condition.condition_type,
                type_react=post_condition.reaction_type,
                const_cond=post_condition.const_cond,
                io_cond=control_msgs.IOCondition(
                    di=self.__to_digital_request_list__(
                        [{'address': di[0], 'state': di[1]} for di in post_condition.digital_inputs]),
                    # di=self.__to_digital_request_list__(post_condition.digital_inputs),
                    # end_di=self.__to_digital_request_list__(post_condition['enddi_condition']),
                ),
            )

        response = self.control.MoveConveyor(control_msgs.MoveConveyorReq(
            teaching_mode=teaching_mode,
            bypass_singular=bypass_singular,
            acc_ratio=acc_ratio,
            post_condition=post_cond
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Move Waypoints
    ############################
    
    def add_joint_waypoint(self, waypoint: list):
        """
        Input:
            waypoint -> float[] (joint angles in deg)
        Output:
            True
        """
        self._joint_waypoint.append(waypoint)
        return True

    def get_joint_waypoint(self):
        """
        Output:
            list of float[] (stored joint waypoints)
        """
        return self._joint_waypoint
    
    def clear_joint_waypoint(self):
        """
        Output:
            True
        """
        self._joint_waypoint.clear()
        return True
    
    def move_joint_waypoint(self, move_time=None):
        """
        Input:
            move_time -> float (seconds, optional)
        Output:
            True
        """
        for wp in self._joint_waypoint:
            if move_time is None:
                self.movej(jtarget = wp, blending_type=BlendingType.OVERRIDE)
            else:
                self.movej_time(jtarget = wp, blending_type=BlendingType.OVERRIDE, move_time=move_time)
            self.wait_progress(progress=100)
        return True

    def add_task_waypoint(self, waypoint: list):
        """
        Input:
            waypoint -> float[6] [mm, mm, mm, deg, deg, deg]
        Output:
            True
        """
        self._task_waypoint.append(waypoint)
        return True
    
    def get_task_waypoint(self):
        """
        Output:
            list of float[6] (stored task waypoints)
        """
        return self._task_waypoint
    
    def clear_task_waypoint(self):
        """
        Output:
            True
        """
        self._task_waypoint.clear()
        return True
    
    def move_task_waypoint(self, move_time=None):
        """
        Input:
            move_time -> float (seconds, optional)
        Output:
            True
        """
        for wp in self._task_waypoint:
            if move_time is None:
                self.movel(ttarget = wp, blending_type=BlendingType.OVERRIDE)
            else:
                self.movel_time(ttarget = wp, blending_type=BlendingType.OVERRIDE, move_time=move_time)
            self.wait_progress(progress=100)
        return True

    ############################
    # Motion Control (Teaching mode)
    ############################
    def move_home(self):
        """
        Output:
            None (moves robot to home position)
        """
        home_pos = self.get_home_pos()['jpos']
        self.movej(home_pos,
                   blending_type=BlendingType.NONE,
                   base_type=JointBaseType.ABSOLUTE,
                   blending_radius=0.0,
                   vel_ratio=Limits.JogVelRatioDefault,
                   acc_ratio=Limits.JogAccRatioDefault,
                   post_condition=PostCondition(),
                   teaching_mode=False)

    ############################
    # Motion Control (Teleoperation)
    ############################
    def get_teleop_device(self):
        """
        Output:
            name -> string
            type -> int32
            ip -> string
            port -> int32
        """
        response = self.control.GetTeleOpDevice(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_teleop_state(self):
        """
        Output:
            mode -> int32 (TeleMode)
            method -> int32 (TeleMethod)
        """
        response = self.control.GetTeleOpState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def connect_teleop_device(self, name: str, type: control_msgs.TeleOpDevice, ip: str, port: int):
        """
        Input:
            name -> string
            type -> int32 (TeleOpDeviceType)
            ip -> string
            port -> int32
        Output:
            response -> {code, msg}
        """
        response = self.control.ConnectTeleOpDevice(
            control_msgs.TeleOpDevice(name=name,type=type,ip=ip,port=port)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def disconnect_teleop_device(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.DisConnectTeleOpDevice(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def read_teleop_input(self):
        """
        Output:
            buttons -> int32[]
            axes -> double[]
        """
        response = self.control.ReadTeleOpInput(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    
    def start_teleop(self, method):
        """
        Input:
            method -> int32 (TELE_TASK_ABSOLUTE=0, TELE_TASK_RELATIVE=1, TELE_JOINT_ABSOLUTE=10, TELE_JOINT_RELATIVE=11)
        Output:
            response -> {code, msg}
        """
        response = self.control.StartTeleOp(
            control_msgs.TeleOpState(mode=control_msgs.TeleMode.TELE_RAW, method=method))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def stop_teleop(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.StopTeleOp(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_play_rate(self, rate: float):
        """
        Input:
            rate -> float
        Output:
            response -> {code, msg}
        """
        response = self.control.SetPlayRate(control_msgs.TelePlayRate(rate=rate))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_play_rate(self):
        """
        Output:
            rate -> float
        """
        response = self.control.GetPlayRate(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tele_file_list(self):
        """
        Output:
            names -> string[]
        """
        response = self.control.GetTeleFileList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def save_tele_motion(self, name: str):
        """
        Input:
            name -> string
        Output:
            response -> {code, msg}
        """
        response = self.control.SaveTeleMotion(control_msgs.TeleFileReq(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def load_tele_motion(self, name: str):
        """
        Input:
            name -> string
        Output:
            response -> {code, msg}
        """
        response = self.control.LoadTeleMotion(control_msgs.TeleFileReq(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def delete_tele_motion(self, name: str):
        """
        Input:
            name -> string
        Output:
            response -> {code, msg}
        """
        response = self.control.DeleteTeleMotion(control_msgs.TeleFileReq(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def enable_tele_key(self, enable):
        """
        Input:
            enable -> bool
        Output:
            response -> {code, msg}
        """
        response = self.control.EnableTeleKey(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movetelej_abs(self, jpos, vel_ratio=0.8, acc_ratio=7.0):
        """
        Input:
            jpos -> float[] (joint angles in deg, absolute)
            vel_ratio -> float
            acc_ratio -> float
        Output:
            response -> {code, msg}
        """
        response = self.control.MoveTeleJ(control_msgs.MoveTeleJReq(jpos=jpos, vel_ratio=vel_ratio, acc_ratio=acc_ratio,
                                                                    method=control_msgs.TELE_JOINT_ABSOLUTE))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movetelej_rel(self, jpos, vel_ratio=0.8, acc_ratio=7.0):
        """
        Input:
            jpos -> float[] (joint angles in deg, relative)
            vel_ratio -> float
            acc_ratio -> float
        Output:
            response -> {code, msg}
        """
        response = self.control.MoveTeleJ(control_msgs.MoveTeleJReq(jpos=jpos, vel_ratio=vel_ratio, acc_ratio=acc_ratio,
                                                                    method=control_msgs.TELE_JOINT_RELATIVE))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movetelel_abs(self, tpos, vel_ratio=0.8, acc_ratio=7.0):
        """
        Input:
            tpos -> float[6] [mm, mm, mm, deg, deg, deg] (absolute)
            vel_ratio -> float
            acc_ratio -> float
        Output:
            response -> {code, msg}
        """
        response = self.control.MoveTeleL(control_msgs.MoveTeleLReq(tpos=tpos, vel_ratio=vel_ratio, acc_ratio=acc_ratio,
                                                                    method=control_msgs.TELE_TASK_ABSOLUTE))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movetelel_rel(self, tpos, vel_ratio=0.8, acc_ratio=7.0):
        """
        Input:
            tpos -> float[6] [mm, mm, mm, deg, deg, deg] (relative)
            vel_ratio -> float
            acc_ratio -> float
        Output:
            response -> {code, msg}
        """
        response = self.control.MoveTeleL(control_msgs.MoveTeleLReq(tpos=tpos, vel_ratio=vel_ratio, acc_ratio=acc_ratio,
                                                                    method=control_msgs.TELE_TASK_RELATIVE))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    
    def move_axis(self, start_mm, target_mm, is_absolute=True, vel_ratio=5, acc_ratio=100, teaching_mode=False):
        """
        Input:
            start_mm -> float[3] (mm)
            target_mm -> float[3] (mm)
            is_absolute -> bool
            vel_ratio -> int (0~100)
            acc_ratio -> int (0~100)
            teaching_mode -> bool
        Output:
            response -> {code, msg}
        """
        # print("Linear Control ====================")
        # print("target_mm ", target_mm)
        # print("is_absolute ", is_absolute)
        # print("vel_ratio ", vel_ratio)
        # print("acc_ratio ", acc_ratio)
        # print("teaching_mode ", teaching_mode)

        # vel = Common.Limits.ExternalMotorSpeedMax * vel_ratio / 100
        vel = 250 * vel_ratio / 100 # 250 mm/s
        acc = vel * acc_ratio / 100
        response = self.control.MoveLinearAxis(control_msgs.MoveAxisReq(
            start_mm=start_mm,
            target_mm=target_mm,
            vel_percentage=vel_ratio,
            acc_percentage=acc_ratio,
            is_absolute=is_absolute,
            teaching_mode=teaching_mode
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Control - Additional
    ############################
    def inverse_kin(self, tpos, init_jpos) -> dict:
        """
        Input:
            tpos -> float[6] [mm, mm, mm, deg, deg, deg]
            init_jpos -> float[] (initial joint position in deg)
        Output:
            jpos -> float[]
        """
        response = self.control.InverseKinematics(control_msgs.InverseKinematicsReq(
            tpos=list(tpos),
            init_jpos=list(init_jpos)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    def forward_kin(self, jpos) -> dict:
        """
        Input:
            jpos -> float[] (joint angles in deg)
        Output:
            tpos -> float[6]
        """
        response = self.control.ForwardKinematics(control_msgs.ForwardKinematicsReq(
            jpos=list(jpos)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_direct_teaching(self, enable=True) -> dict:
        """
        Input:
            enable -> bool
        Output:
            response -> {code, msg}
        """
        response = self.control.SetDirectTeaching(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_simulation_mode(self, enable=True) -> dict:
        """
        Input:
            enable -> bool
        Output:
            response -> {code, msg}
        """
        response = self.control.SetSimulationMode(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def recover(self) -> dict:
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.Recover(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_manual_recovery(self, enable=True) -> dict:
        """
        Input:
            enable -> bool
        Output:
            response -> {code, msg}
        """
        response = self.control.SetManualRecovery(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def calculate_relative_pose(self, start_pos, end_pos,
                                base_type=TaskBaseType.ABSOLUTE):
        """
        Input:
            start_pos -> float[6]
            end_pos -> float[6]
            base_type -> TaskBaseType (ABSOLUTE=0, RELATIVE=1, TCP=2)
        Output:
            tpos -> float[6]
        """
        response = self.control.CalculateRelativePose(control_msgs.CalculateRelativePoseReq(
            start_pos=list(start_pos),
            end_pos=list(end_pos),
            base_type=base_type
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def calculate_current_pose_rel(self, current_pos, relative_pos,
                                   base_type=TaskBaseType.ABSOLUTE):
        """
        Input:
            current_pos -> float[6]
            relative_pos -> float[6]
            base_type -> TaskBaseType (ABSOLUTE=0, RELATIVE=1, TCP=2)
        Output:
            tpos -> float[6]
        """
        response = self.control.CalculateCurrentPoseRel(control_msgs.CalculateCurrentPoseRelReq(
            current_pos=list(current_pos),
            relative_pos=list(relative_pos),
            base_type=base_type
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Program control
    ############################
    def play_program(self, prog_name: str = '', prog_idx: int = -1):
        """
        Input:
            prog_name -> string
            prog_idx -> int32
        Output:
            response -> {code, msg}
        """
        response = self.control.PlayProgram(control_msgs.Program(
            prog_name=prog_name,
            prog_idx=prog_idx
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def pause_program(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.PauseProgram(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def resume_program(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.ResumeProgram(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def stop_program(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.StopProgram(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tact_time(self, type: str, tact_time: float):
        """
        Input:
            type -> string
            tact_time -> float (seconds)
        Output:
            response -> {code, msg}
        """
        response = self.control.SetTactTime(common_msgs.TactTime(
            type=type, tact_time=tact_time
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tact_time(self):
        """
        Output:
            type -> string
            tact_time -> float (seconds)
        """
        response = self.control.GetTactTime(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_speed_ratio(self, speed_ratio: int):
        """
        Input:
            speed_ratio -> uint32 (0~100)
        Output:
            response -> {code, msg}
        """
        response = self.config.SetSpeedRatio(config_msgs.Ratio(
            ratio=speed_ratio
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Compliance Mode
    ############################
    def set_compliance_mode(self, enable: bool, stiffness: 'Optional[List[int]]' = None):
        """
        Input:
            enable -> bool
            stiffness -> int32[] (per-axis stiffness levels)
        Output:
            response -> {code, msg}
        """
        response = self.control.SetComplianceMode(
            control_msgs.ComplianceMode(
                enable=enable,
                stiffness=stiffness or []
            )
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_compliance_mode(self):
        """
        Output:
            enable -> bool
            stiffness -> int32[]
        """
        response = self.control.GetComplianceMode(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Variables
    ############################

    # def get_modbus_variable(self):
    #     """
    #     Modbus Variables:
    #         [
    #             name -> string
    #             addr -> int32
    #             value -> int32
    #         ]
    #     """
    #     response = self.control.GetModbusVariable(common_msgs.Empty())
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)['variables']
    
    def get_bool_variable(self):
        """
        Output:
            variables -> [{addr: int32, value: bool}]
        """
        response = self.control.GetBoolVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_int_variable(self):
        """
        Output:
            variables -> [{addr: int32, value: int32}]
        """
        response = self.control.GetIntVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_float_variable(self):
        """
        Output:
            variables -> [{addr: int32, value: float}]
        """
        response = self.control.GetFloatVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_jpos_variable(self):
        """
        Output:
            variables -> [{addr: int32, jpos: float[]}]
        """
        response = self.control.GetJPosVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)['variables']

    def get_tpos_variable(self):
        """
        Output:
            variables -> [{addr: int32, tpos: float[]}]
        """
        response = self.control.GetTPosVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    # def set_modbus_variable(self, modbus_variables: list):
    #     """
    #     Modbus Variables:
    #         [
    #             name -> string
    #             addr -> int32
    #             value -> int32
    #         ]
    #     """
    #     variable_list = []
    #     for modbus_var in modbus_variables:
    #         variable_list.append(control_msgs.ModbusVariable(name=modbus_var['name'], addr=modbus_var['addr'],
    #                                                          value=modbus_var['value'],
    #                                                          signal_type=modbus_var['signal_type']))
    #     response = self.control.SetModbusVariable(
    #         control_msgs.ModbusVars(variables=variable_list)
    #     )
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    def set_bool_variable(self, bool_variables: list):
        """
        Input:
            bool_variables -> [{addr: int32, value: bool}]
        Output:
            response -> {code, msg}
        """
        variable_list = []
        for bool_var in bool_variables:
            variable_list.append(control_msgs.BoolVariable(addr=bool_var['addr'], value=bool_var['value']))

        response = self.control.SetBoolVariable(
            control_msgs.BoolVars(variables=variable_list)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_int_variable(self, int_variables: list):
        """
        Input:
            int_variables -> [{addr: int32, value: int64}]
        Output:
            response -> {code, msg}
        """
        variable_list = []
        for int_var in int_variables:
            variable_list.append(control_msgs.IntVariable(addr=int_var['addr'], value=int_var['value']))

        response = self.control.SetIntVariable(
            control_msgs.IntVars(variables=variable_list)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_float_variable(self, float_variables: list):
        """
        Input:
            float_variables -> [{addr: int32, value: float}]
        Output:
            response -> {code, msg}
        """
        variable_list = []
        for float_var in float_variables:
            variable_list.append(control_msgs.FloatVariable(addr=float_var['addr'], value=float_var['value']))

        response = self.control.SetFloatVariable(
            control_msgs.FloatVars(variables=variable_list)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_jpos_variable(self, jpos_variables: list):
        """
        Input:
            jpos_variables -> [{addr: int32, jpos: float[]}]
        Output:
            response -> {code, msg}
        """
        variable_list = []
        for jpos in jpos_variables:
            variable_list.append(control_msgs.JPosVariable(addr=jpos['addr'], jpos=jpos['jpos']))

        response = self.control.SetJPosVariable(
            control_msgs.JPosVars(variables=variable_list)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tpos_variable(self, tpos_variables: list):
        """
        Input:
            tpos_variables -> [{addr: int32, tpos: float[]}]
        Output:
            response -> {code, msg}
        """
        variable_list = []
        for tpos in tpos_variables:
            variable_list.append(control_msgs.TPosVariable(addr=tpos['addr'], tpos=tpos['tpos']))

        response = self.control.SetTPosVariable(
            control_msgs.TPosVars(variables=variable_list)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Plugin Variables
    ############################
    def set_plugin_bool_variable(self, name: str, value: bool):
        """
        Input:
            name -> string
            value -> bool
        Output:
            response -> {code, msg}
        """
        response = self.control.SetPluginBoolVariable(
            common_msgs.NamedBool(name=name, value=value)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plugin_bool_variable(self, name: str):
        """
        Input:
            name -> string
        Output:
            name -> string
            value -> bool
        """
        response = self.control.GetPluginBoolVariable(
            common_msgs.Name(name=name)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_plugin_int_variable(self, name: str, value: int):
        """
        Input:
            name -> string
            value -> int64
        Output:
            response -> {code, msg}
        """
        response = self.control.SetPluginIntVariable(
            common_msgs.NamedInt(name=name, value=value)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plugin_int_variable(self, name: str):
        """
        Input:
            name -> string
        Output:
            name -> string
            value -> int64
        """
        response = self.control.GetPluginIntVariable(
            common_msgs.Name(name=name)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_plugin_float_variable(self, name: str, value: float):
        """
        Input:
            name -> string
            value -> float
        Output:
            response -> {code, msg}
        """
        response = self.control.SetPluginFloatVariable(
            common_msgs.NamedFloat(name=name, value=value)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plugin_float_variable(self, name: str):
        """
        Input:
            name -> string
        Output:
            name -> string
            value -> float
        """
        response = self.control.GetPluginFloatVariable(
            common_msgs.Name(name=name)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_plugin_jpos_variable(self, name: str, jpos: List[float]):
        """
        Input:
            name -> string
            jpos -> float[]
        Output:
            response -> {code, msg}
        """
        response = self.control.SetPluginJPosVariable(
            common_msgs.NamedJointPosition(name=name, jpos=jpos)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plugin_jpos_variable(self, name: str):
        """
        Input:
            name -> string
        Output:
            name -> string
            jpos -> float[]
        """
        response = self.control.GetPluginJPosVariable(
            common_msgs.Name(name=name)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_plugin_tpos_variable(self, name: str, tpos: List[float]):
        """
        Input:
            name -> string
            tpos -> float[]
        Output:
            response -> {code, msg}
        """
        response = self.control.SetPluginTPosVariable(
            common_msgs.NamedTaskPosition(name=name, tpos=tpos)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plugin_tpos_variable(self, name: str):
        """
        Input:
            name -> string
        Output:
            name -> string
            tpos -> float[]
        """
        response = self.control.GetPluginTPosVariable(
            common_msgs.Name(name=name)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Config
    ############################
    def get_pack_pos(self):
        """
        Output:
            jpos -> double[]
        """
        response = self.config.GetPackPosition(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_path_config(self):
        """
        Output:
            path config dict
        """
        response = self.config.GetPathConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    # def get_nonce(self):
    #     response = self.config.GetNonce(common_msgs.Empty())
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    # def login(self, digest: dict):
    #     req = config_msgs.Digest()
    #     ParseDict(digest, req)
    #     response = self.config.Login(req)
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    # def test_digest(self, passwd: dict):
    #     req = config_msgs.Passwd()
    #     ParseDict(passwd, req)
    #     response = self.config.TestDigest(req)
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    # def verify_token(self, token: dict):
    #     req = config_msgs.Token()
    #     ParseDict(token, req)
    #     response = self.config.VerifyToken(req)
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    # def change_password(self, request: dict):
    #     req = config_msgs.ChangePasswordReq()
    #     ParseDict(request, req)
    #     response = self.config.ChangePassword(req)
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    def get_joint_limit_config(self):
        """
        Output:
            joint_pos_limits -> {upper: double[], lower: double[]}
            joint_vel_limits -> double[]
            joint_acc_limits -> double[]
            joint_torque_limits -> double[]
        """
        response = self.config.GetJointLimitConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_joint_limit_config(self, config: dict):
        """
        Input:
            config -> dict (JointLimitConfig schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.JointLimitConfig()
        ParseDict(config, req)
        response = self.config.SetJointLimitConfig(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_original_joint_limit_config(self):
        """
        Output:
            joint_pos_limits -> {upper: double[], lower: double[]}
            joint_vel_limits -> double[]
            joint_acc_limits -> double[]
            joint_torque_limits -> double[]
        """
        response = self.config.GetOriginalJointLimitConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_operation_mode_config(self):
        """
        Output:
            operation mode config dict
        """
        response = self.config.GetOperationModeConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_operation_mode_config(self, config: dict):
        """
        Input:
            config -> dict (OperationModeConfig schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.OperationModeConfig()
        ParseDict(config, req)
        response = self.config.SetOperationModeConfig(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_new_controller_test_on_off_state(self):
        """
        Output:
            enable -> bool
        """
        response = self.config.GetNewControllerTestOnOffState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_new_controller_test_on_off(self, state: dict):
        """
        Input:
            state -> dict (NewControllerTestState schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.NewControllerTestState()
        ParseDict(state, req)
        response = self.config.SetNewControllerTestOnOff(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_test_control_gain(self):
        """
        Output:
            test gain set dict
        """
        response = self.config.GetTestControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_test_control_gain(self, gains: dict):
        """
        Input:
            gains -> dict (TestGainSet schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.TestGainSet()
        ParseDict(gains, req)
        response = self.config.SetTestControlGain(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_property_list(self):
        """
        Output:
            entries -> [{name: string, mass: float, center_of_mass: float[3], inertia: float[6]}]
        """
        response = self.config.GetToolPropertyList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_property_list(self, tool_properties: dict):
        """
        Input:
            tool_properties -> dict (ToolPropertyEntries schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.ToolPropertyEntries()
        ParseDict(tool_properties, req)
        response = self.config.SetToolPropertyList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_weld_position_list(self):
        """
        Output:
            weld position list dict
        """
        response = self.config.GetWeldPositionList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_weld_position_list(self, weld_positions: dict):
        """
        Input:
            weld_positions -> dict (WeldPositionList schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.WeldPositionList()
        ParseDict(weld_positions, req)
        response = self.config.SetWeldPositionList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_welding_machine_config(self):
        """
        Output:
            welding config info dict
        """
        response = self.config.GetWeldingMachineConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_welding_machine_config(self, config: dict):
        """
        Input:
            config -> dict (WeldingConfigInfo schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.WeldingConfigInfo()
        ParseDict(config, req)
        response = self.config.SetWeldingMachineConfig(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def list_safety_snapshots(self):
        """
        Output:
            snapshots -> [{id: string, name: string, ...}]
        """
        response = self.config.ListSafetySnapshots(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def save_safety_snapshot(self, request: dict):
        """
        Input:
            request -> dict (SaveSafetySnapshotReq schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.SaveSafetySnapshotReq()
        ParseDict(request, req)
        response = self.config.SaveSafetySnapshot(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def restore_safety_snapshot(self, snapshot_id: dict):
        """
        Input:
            snapshot_id -> dict (SafetySnapshotId schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.SafetySnapshotId()
        ParseDict(snapshot_id, req)
        response = self.config.RestoreSafetySnapshot(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def delete_safety_snapshot(self, snapshot_id: dict):
        """
        Input:
            snapshot_id -> dict (SafetySnapshotId schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.SafetySnapshotId()
        ParseDict(snapshot_id, req)
        response = self.config.DeleteSafetySnapshot(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def restore_factory_control_gains(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.config.RestorFactoryControlGains(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def restore_factory_safety_config(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.config.RestorFactorySafetyConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_imu_auto_mount(self):
        """
        Output:
            ry -> float
            rz -> float
        """
        response = self.config.GetIMUAutoMount(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_locked_joint(self, index: int):
        """
        Input:
            index -> int32
        Output:
            response -> {code, msg}
        """
        response = self.config.SetLockedJoint(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_link(self, index: int):
        """
        Input:
            index -> int32
        Output:
            response -> {code, msg}
        """
        response = self.config.SetToolLink(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_speed_ratio(self):
        """
        Output:
            ratio -> uint32
        """
        response = self.config.GetSpeedRatio(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_list(self, tool_list: dict):
        """
        Input:
            tool_list -> dict (ToolList schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.ToolList()
        ParseDict(tool_list, req)
        response = self.config.SetToolList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_list(self):
        """
        Output:
            tools -> [{name: string, ...}]
        """
        response = self.config.GetToolList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_vision_server_list(self):
        """
        Output:
            vision_servers -> [{name: string, ip: string, port: int32, ...}]
        """
        response = self.config.GetVisionServerList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_vision_server_list(self, vision_server_list: dict):
        """
        Input:
            vision_server_list -> dict (VisionServerList schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.VisionServerList()
        ParseDict(vision_server_list, req)
        response = self.config.SetVisionServerList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_modbus_server_list(self):
        """
        Output:
            modbus_servers -> [{name: string, ip: string, port: int32, ...}]
        """
        response = self.config.GetModbusServerList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_modbus_server_list(self, modbus_server_list: dict):
        """
        Input:
            modbus_server_list -> dict (ModbusServerList schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.ModbusServerList()
        ParseDict(modbus_server_list, req)
        response = self.config.SetModbusServerList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_conveyor_list(self):
        """
        Output:
            conveyors -> [{name: string, ...}]
        """
        response = self.config.GetConveyorList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_list(self, conveyor_list: dict):
        """
        Input:
            conveyor_list -> dict (ConveyorList schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.ConveyorList()
        ParseDict(conveyor_list, req)
        response = self.config.SetConveyorList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_compliance_control_joint_gain(self, gains: dict):
        """
        Input:
            gains -> dict (ComplianceGainSet schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.ComplianceGainSet()
        ParseDict(gains, req)
        response = self.config.SetComplianceControlJointGain(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_compliance_control_joint_gain(self):
        """
        Output:
            compliance gain set dict
        """
        response = self.config.GetComplianceControlJointGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_frame_list(self):
        """
        Output:
            tool_frames -> [{name: string, fpos: float[6]}]
            default_name -> string
        """
        response = self.config.GetToolFrameList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_frame_list(self, tool_frame_list: dict):
        """
        Input:
            tool_frame_list -> dict (ToolFrameList schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.ToolFrameList()
        ParseDict(tool_frame_list, req)
        response = self.config.SetToolFrameList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ref_frame_list(self):
        """
        Output:
            ref_frames -> [{name: string, tpos: float[], ...}]
            default_name -> string
        """
        response = self.config.GetRefFrameList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_ref_frame_list(self, ref_frame_list: dict):
        """
        Input:
            ref_frame_list -> dict (RefFrameList schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.RefFrameList()
        ParseDict(ref_frame_list, req)
        response = self.config.SetRefFrameList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_custom_pos_list(self):
        """
        Output:
            custom_pos list dict
        """
        response = self.config.GetCustomPosList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_custom_pos_list(self, custom_pos_list: dict):
        """
        Input:
            custom_pos_list -> dict (CustomPosList schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.CustomPosList()
        ParseDict(custom_pos_list, req)
        response = self.config.SetCustomPosList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_shape_list(self, tool_shape_list: dict):
        """
        Input:
            tool_shape_list -> dict (ToolShapeList schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.ToolShapeList()
        ParseDict(tool_shape_list, req)
        response = self.config.SetToolShapeList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_shape_list(self):
        """
        Output:
            tool shape list dict
        """
        response = self.config.GetToolShapeList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_environment_list(self, environment_list: dict):
        """
        Input:
            environment_list -> dict (EnvironmentList schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.EnvironmentList()
        ParseDict(environment_list, req)
        response = self.config.SetEnvironmentList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_environment_list(self):
        """
        Output:
            environment list dict
        """
        response = self.config.GetEnvironmentList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_default_coll_sens_param(self):
        """
        Output:
            default collision sensitivity params dict
        """
        response = self.config.GetDefaultCollSensParam(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_sensorless_params(self, params: dict):
        """
        Input:
            params -> dict (SensorlessParams schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.SensorlessParams()
        ParseDict(params, req)
        response = self.config.SetSensorlessParams(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_sensorless_params(self):
        """
        Output:
            sensorless params dict
        """
        response = self.config.GetSensorlessParams(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_on_start_program_config(self, config: dict):
        """
        Input:
            config -> dict (OnStartProgramConfig schema)
        Output:
            response -> {code, msg}
        """
        req = config_msgs.OnStartProgramConfig()
        ParseDict(config, req)
        response = self.config.SetOnStartProgramConfig(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_on_start_program_config(self):
        """
        Output:
            on-start program config dict
        """
        response = self.config.GetOnStartProgramConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_simple_coll_threshold(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.config.SetSimpleCollThreshold(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_collison_model_margin(self):
        """
        Output:
            collision_margin -> float
            recover_margin -> float
        """
        response = self.config.GetCollisonModelMargin(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_collison_model_margin(self, collision_margin: float, recover_margin: float):
        """
        Input:
            collision_margin -> float
            recover_margin -> float
        Output:
            response -> {code, msg}
        """
        response = self.config.SetCollisonModelMargin(
            config_msgs.CollisionModelMargin(collision_margin=collision_margin,
                                             recover_margin=recover_margin)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Bus Events
    ############################
    def push_bus_event(self, event_id: int,
                       b_data: 'Optional[List[bool]]' = None,
                       i_data: 'Optional[List[int]]' = None,
                       f_data: 'Optional[List[float]]' = None,
                       text_data: 'Optional[str]' = None):
        """
        Input:
            event_id -> int32
            b_data -> bool[] (optional)
            i_data -> int64[] (optional)
            f_data -> double[] (optional)
            text_data -> string (optional)
        Output:
            response -> {code, msg}
        """
        evt = control_msgs.BusEvent(
            event_id=event_id,
            b_data=b_data or [],
            i_data=i_data or [],
            f_data=f_data or [],
            text_data=text_data or ""
        )
        response = self.control.PushBusEvent(evt)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def catch_bus_event(self, event_id: int, timeout: float):
        """
        Input:
            event_id -> int32
            timeout -> float (seconds)
        Output:
            event_id -> int32
            b_data -> bool[]
            i_data -> int64[]
            f_data -> double[]
            text_data -> string
        """
        req = control_msgs.CatchBusEventReq(event_id=event_id, timeout=timeout)
        response = self.control.CatchBusEvent(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Force Mode
    ############################
    def set_force_mode(self, force_mode: dict):
        """
        Input:
            force_mode -> dict (ForceModeReq schema)
        Output:
            response -> {code, msg}
        """
        msg = control_msgs.ForceModeReq()
        ParseDict(force_mode, msg)
        response = self.control.SetForceMode(msg)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_force_mode(self):
        """
        Output:
            force mode dict (ForceModeReq schema)
        """
        response = self.control.GetForceMode(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
        
        
    def get_home_pos(self):
        """
        Output:
            jpos -> double[]
        """
        response = self.config.GetHomePosition(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_home_pos(self, home_jpos: list):
        """
        Input:
            home_jpos -> double[] (joint positions in deg)
        Output:
            response -> {code, msg}
        """
        response = self.config.SetHomePosition(config_msgs.JointPos(
            jpos=home_jpos
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ref_frame(self):
        """
        Output:
            fpos -> float[6]
        """

        response = self.config.GetRefFrame(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_ref_frame(self, fpos: list):
        """
        Input:
            fpos -> float[6]
        Output:
            response -> {code, msg}
        """
        response = self.config.SetRefFrame(config_msgs.Frame(
            fpos=list(fpos)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_ref_frame_planar(self, fpos0: list, fpos1: list, fpos2: list):
        """
        Input:
            fpos0 -> float[6]
            fpos1 -> float[6]
            fpos2 -> float[6]
        Output:
            response -> {code, msg}
        """
        response = self.config.SetRefFramePlanar(config_msgs.PlanarFrame(
            fpos0=list(fpos0), fpos1=list(fpos1), fpos2=list(fpos2)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_frame(self, fpos: list):
        """
        Input:
            fpos -> float[6]
        Output:
            response -> {code, msg}
        """
        response = self.config.SetToolFrame(config_msgs.Frame(
            fpos=list(fpos)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_friction_comp(self):
        """
        Output:
            joint_idx -> uint32
            control_comp_enable -> bool
            control_comp_levels -> int32[6]
            teaching_comp_enable -> bool
            teaching_comp_levels -> int32[6]
        """
        response = self.config.GetFrictionComp(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_friction_comp(self, control_comp: bool, control_comp_levels: list,
                          dt_comp: bool, dt_comp_levels: list):
        """
        Input:
            control_comp -> bool
            control_comp_levels -> int32[6]
            dt_comp -> bool (teaching_comp_enable)
            dt_comp_levels -> int32[6] (teaching_comp_levels)
        Output:
            response -> {code, msg}
        """
        response = self.config.SetFrictionComp(config_msgs.FrictionCompSet(
            control_comp_enable=control_comp, control_comp_levels=list(control_comp_levels),
            teaching_comp_enable=dt_comp, teaching_comp_levels=list(dt_comp_levels)
        ))

        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_friction_comp_state(self, enable=False) -> dict:
        """
        Input:
            enable -> bool
        Output:
            response -> {code, msg}
        """
        response = self.control.SetFrictionCompensation(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_friction_comp_state(self) -> dict:
        """
        Output:
            enable -> bool
        """
        response = self.control.GetFrictionCompensationState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_mount_pos(self, rot_y=0.0, rot_z=0.0):
        """
        Input:
            rot_y -> float (degrees)
            rot_z -> float (degrees)
        Output:
            response -> {code, msg}
        """
        response = self.config.SetMountPos(config_msgs.MountingAngles(
            ry=rot_y, rz=rot_z
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_mount_pos(self):
        """
        Output:
            ry -> float
            rz -> float
        """
        response = self.config.GetMountPos(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_property(self):
        """
        Output:
            mass -> float
            center_of_mass -> float[3]
            inertia -> float[6]
        """
        response = self.config.GetToolProperty(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_property(self, mass: float, center_of_mass: list, inertia: list):
        """
        Input:
            mass -> float
            center_of_mass -> float[3]
            inertia -> float[6]
        Output:
            response -> {code, msg}
        """
        response = self.config.SetToolProperty(config_msgs.ToolProperties(
            mass=mass, center_of_mass=list(center_of_mass), inertia=list(inertia)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_coll_sens_level(self):
        """
        Output:
            level -> uint32
        """
        response = self.config.GetCollSensLevel(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_coll_sens_level(self, level: int):
        """
        Input:
            level -> uint32
        Output:
            response -> {code, msg}
        """
        response = self.config.SetCollSensLevel(config_msgs.CollisionSensLevel(
            level=level
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_coll_sens_param(self):
        """
        Output:
            j_torque_bases -> double[6]
            j_torque_tangents -> double[6]
            t_torque_bases -> double[6]
            t_torque_tangents -> double[6]
            error_bases -> double[6]
            error_tangents -> double[6]
            t_constvel_torque_bases -> double[6]
            t_constvel_torque_tangents -> double[6]
            t_conveyor_torque_bases -> double[6]
            t_conveyor_torque_tangents -> double[6]
        """
        response = self.config.GetCollSensParam(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_coll_sens_param(self, j_torque_bases, j_torque_tangents,
                            t_torque_bases, t_torque_tangents,
                            t_constvel_torque_bases, t_constvel_torque_tangents,
                            t_conveyor_torque_bases, t_conveyor_torque_tangents,
                            error_bases, error_tangents):
        """
        Input:
            j_torque_bases -> double[6]
            j_torque_tangents -> double[6]
            t_torque_bases -> double[6]
            t_torque_tangents -> double[6]
            t_constvel_torque_bases -> double[6]
            t_constvel_torque_tangents -> double[6]
            t_conveyor_torque_bases -> double[6]
            t_conveyor_torque_tangents -> double[6]
            error_bases -> double[6]
            error_tangents -> double[6]
        Output:
            response -> {code, msg}
        """
        response = self.config.SetCollSensParam(config_msgs.CollisionThresholds(
            j_torque_bases=list(j_torque_bases), j_torque_tangents=list(j_torque_tangents),
            t_torque_bases=list(t_torque_bases), t_torque_tangents=list(t_torque_tangents),
            error_bases=list(error_bases), error_tangents=list(error_tangents),
            t_constvel_torque_bases=list(t_constvel_torque_bases),
            t_constvel_torque_tangents=list(t_constvel_torque_tangents),
            t_conveyor_torque_bases=list(t_conveyor_torque_bases),
            t_conveyor_torque_tangents=list(t_conveyor_torque_tangents)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_coll_policy(self):
        """
        Output:
            policy -> uint32
            sleep_time -> float
            gravity_time -> float
        """
        response = self.config.GetCollPolicy(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_coll_policy(self, policy=CollisionPolicyType.NONE,
                        sleep_time=0, gravity_time=0.1):
        """
        Input:
            policy -> uint32 (CollisionPolicyType)
            sleep_time -> float
            gravity_time -> float
        Output:
            response -> {code, msg}
        """
        CollisionPolicyType.NONE
        response = self.config.SetCollPolicy(config_msgs.CollisionPolicy(
            policy=policy, sleep_time=sleep_time, gravity_time=gravity_time
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_safety_limits(self):
        """
        Output:
            power_limit -> float
            power_limit_ratio -> float
            tcp_force_limit -> float
            tcp_force_limit_ratio -> float
            tcp_speed_limit -> float
            tcp_speed_limit_ratio -> float
            joint_upper_limits -> float[]
            joint_lower_limits -> float[]
        """
        response = self.config.GetSafetyLimits(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_safety_limits(self, power_limit: float, power_limit_ratio: float,
                          tcp_force_limit: float, tcp_force_limit_ratio: float,
                          tcp_speed_limit: float, tcp_speed_limit_ratio: float):
        """
        Input:
            power_limit -> float
            power_limit_ratio -> float
            tcp_force_limit -> float
            tcp_force_limit_ratio -> float
            tcp_speed_limit -> float
            tcp_speed_limit_ratio -> float
        Output:
            response -> {code, msg}
        """
        response = self.config.SetSafetyLimits(config_msgs.SafetyLimits(
            power_limit=power_limit, power_limit_ratio=power_limit_ratio,
            tcp_force_limit=tcp_force_limit, tcp_force_limit_ratio=tcp_force_limit_ratio,
            tcp_speed_limit=tcp_speed_limit, tcp_speed_limit_ratio=tcp_speed_limit_ratio
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # IndySDK related
    ############################
    def activate_sdk(self, license_key, expire_date):
        """
        Input:
            license_key -> string
            expire_date -> string (YYYY-MM-DD)
        Output:
            activated -> bool
            response -> {code, msg}
        """
        response = self.control.ActivateIndySDK(
            control_msgs.SDKLicenseInfo(license_key=license_key, expire_date=expire_date))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_custom_control_mode(self, mode):
        """
        Input:
            mode -> int32 (0: default controller, 1: IndySDK component)
        Output:
            response -> {code, msg}
        """
        response = self.control.SetCustomControlMode(common_msgs.IntMode(mode=mode))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_custom_control_mode(self):
        """
        Output:
            mode -> int32
        """
        response = self.control.GetCustomControlMode(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_custom_control_gain(self):
        """
        Output:
            gain0 -> float[6]
            gain1 -> float[6]
            gain2 -> float[6]
            gain3 -> float[6]
            gain4 -> float[6]
            gain5 -> float[6]
            gain6 -> float[6]
            gain7 -> float[6]
            gain8 -> float[6]
            gain9 -> float[6]
        """
        response = self.config.GetCustomControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    
    def set_custom_control_gain(self, gain0=None, gain1=None, gain2=None, gain3=None, gain4=None, 
                                gain5=None, gain6=None, gain7=None, gain8=None, gain9=None):
        """
        Input:
            gain0~gain9 -> float[6] each (None defaults to [0,0,0,0,0,0])
        Output:
            response -> {code, msg}
        """
        # Replace None with a list of six 0s
        gains = [gain if gain is not None else [0] * 6 for gain in [gain0, gain1, gain2, gain3, gain4, gain5, gain6, gain7, gain8, gain9]]

        response = self.config.SetCustomControlGain(config_msgs.CustomGainSet(
            gain0=gains[0], gain1=gains[1], gain2=gains[2], gain3=gains[3],
            gain4=gains[4], gain5=gains[5], gain6=gains[6], gain7=gains[7],
            gain8=gains[8], gain9=gains[9]
        ))

        return json_format.MessageToDict(response,
                                        including_default_value_fields=True,
                                        preserving_proto_field_name=True,
                                        use_integers_for_enums=True)

    def set_joint_control_gain(self, kp: list, kv: list, kl2: list):
        """
        Input:
            kp -> float[6]
            kv -> float[6]
            kl2 -> float[6]
        Output:
            response -> {code, msg}
        """
        response = self.config.SetJointControlGain(config_msgs.JointGainSet(
            kp=list(kp), kv=list(kv), kl2=list(kl2)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_joint_control_gain(self):
        """
        Output:
            kp -> float[6]
            kv -> float[6]
            kl2 -> float[6]
        """
        response = self.config.GetJointControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_task_control_gain(self, kp, kv, kl2):
        """
        Input:
            kp -> float[6]
            kv -> float[6]
            kl2 -> float[6]
        Output:
            response -> {code, msg}
        """
        response = self.config.SetTaskControlGain(config_msgs.TaskGainSet(
            kp=list(kp), kv=list(kv), kl2=list(kl2)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_task_control_gain(self):
        """
        Output:
            kp -> float[6]
            kv -> float[6]
            kl2 -> float[6]
        """
        response = self.config.GetTaskControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_impedance_control_gain(self, mass, damping, stiffness, kl2):
        """
        Input:
            mass -> float[6]
            damping -> float[6]
            stiffness -> float[6]
            kl2 -> float[6]
        Output:
            response -> {code, msg}
        """
        response = self.config.SetImpedanceControlGain(config_msgs.ImpedanceGainSet(
            mass=list(mass), damping=list(damping), stiffness=list(stiffness), kl2=list(kl2)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_impedance_control_gain(self):
        """
        Output:
            mass -> float[6]
            damping -> float[6]
            stiffness -> float[6]
            kl2 -> float[6]
        """
        response = self.config.GetImpedanceControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_force_control_gain(self, kp, kv, kl2, mass, damping, stiffness, kpf, kif):
        """
        Input:
            kp -> float[6]
            kv -> float[6]
            kl2 -> float[6]
            mass -> float[6]
            damping -> float[6]
            stiffness -> float[6]
            kpf -> float[6]
            kif -> float[6]
        Output:
            response -> {code, msg}
        """
        response = self.config.SetForceControlGain(config_msgs.ForceGainSet(
            kp=list(kp), kv=list(kv), kl2=list(kl2), mass=list(mass), damping=list(damping), stiffness=list(stiffness),
            kpf=list(kpf), kif=list(kif)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_force_control_gain(self):
        """
        Output:
            kp -> float[6]
            kv -> float[6]
            kl2 -> float[6]
            mass -> float[6]
            damping -> float[6]
            stiffness -> float[6]
            kpf -> float[6]
            kif -> float[6]
        """
        response = self.config.GetForceControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Utility functions
    ############################
    def start_log(self):
        """
        Output:
            None (sets int variable 300 = 1 to start RT logging)
        """
        int_vars_to_set = [{"addr": 300, "value": 1}]
        self.set_int_variable(int_vars_to_set)

    def end_log(self):
        """
        Output:
            None (sets int variable 300 = 2, saves to /home/user/release/IndyDeployments/RTlog/RTLog.csv)
        """
        int_vars_to_set = [{"addr": 300, "value": 2}]
        self.set_int_variable(int_vars_to_set)

    def wait_for_operation_state(self, wait_op_state=None):
        """
        Input:
            wait_op_state -> int (operation state to wait for, or None)
        Output:
            None (blocks until op_state matches)
        """
        if wait_op_state is not None:
            while self.get_robot_data()['op_state'] != wait_op_state:
                time.sleep(0.01)
                
    def wait_for_motion_state(self, wait_motion_state=None): 
        """
        Input:
            wait_motion_state -> string (is_in_motion, is_target_reached, is_pausing, is_stopping, has_motion)
        Output:
            None (blocks until motion state is True)
        """
        motion_list = ["is_in_motion", "is_target_reached", "is_pausing", "is_stopping", "has_motion"]
        if wait_motion_state is not None and wait_motion_state in motion_list:
            while self.get_motion_data()[wait_motion_state] is False:
                time.sleep(0.01)

    ############################
    def wait_io(self, 
                di_signal_list, 
                do_signal_list, 
                end_di_signal_list, 
                end_do_signal_list, 
                conjunction=0):
        """
        Input:
            di_signal_list -> [{address: int32, state: int32}]
            do_signal_list -> [{address: int32, state: int32}]
            end_di_signal_list -> [{address: int32, state: int32}]
            end_do_signal_list -> [{address: int32, state: int32}]
            conjunction -> int32 (0: OR, 1: AND)
        Output:
            response -> {code, msg}
        """
        response = self.control.WaitIO(control_msgs.WaitIOReq(
            di_list=self.__to_digital_request_list__(di_signal_list),
            do_list=self.__to_digital_request_list__(do_signal_list),
            end_di_list=self.__to_digital_request_list__(end_di_signal_list),
            end_do_list=self.__to_digital_request_list__(end_do_signal_list),
            conjunction=conjunction
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    
    
    def wait_time(self, time: float):
        """
        Input:
            time -> float (seconds)
        Output:
            response -> {code, msg}
        """
        response = self.control.WaitTime(control_msgs.WaitTimeReq(
            time=time
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def wait_progress(self, progress: int):
        """
        Input:
            progress -> int32 (0~100 percent)
        Output:
            response -> {code, msg}
        """
        response = self.control.WaitProgress(control_msgs.WaitProgressReq(
            progress=progress
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def wait_traj(self, traj_condition):
        """
        Input:
            traj_condition -> int32
        Output:
            response -> {code, msg}
        """
        response = self.control.WaitTraj(control_msgs.WaitTrajReq(
            traj_condition=traj_condition
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def wait_radius(self, radius: int):
        """
        Input:
            radius -> int32 (mm)
        Output:
            response -> {code, msg}
        """
        response = self.control.WaitRadius(control_msgs.WaitRadiusReq(
            radius=radius
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_io_variable(self):
        """
        Output:
            variables -> IOVars dict
        """
        response = self.control.GetIOVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_io_variable(self, io_vars: dict):
        """
        Input:
            io_vars -> dict (IOVars schema)
        Output:
            response -> {code, msg}
        """
        req = control_msgs.IOVars()
        ParseDict(io_vars, req)
        response = self.control.SetIOVariable(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_variable_name_list(self):
        """
        Output:
            all variables name list dict
        """
        response = self.control.GetVariableNameList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_variable_name_list(self, variables: dict):
        """
        Input:
            variables -> dict (AllVars schema)
        Output:
            response -> {code, msg}
        """
        req = control_msgs.AllVars()
        ParseDict(variables, req)
        response = self.control.SetVariableNameList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    # def set_modbus_variable_name_list(self, variables: dict):
    #     req = control_msgs.ModbusVariableList()
    #     ParseDict(variables, req)
    #     response = self.control.SetModbusVariableNameList(req)
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    def get_program_breakpoints(self):
        """
        Output:
            breakpoints dict
        """
        response = self.control.GetProgramBreakPoints(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_program_breakpoints(self, breakpoints: dict):
        """
        Input:
            breakpoints -> dict (ProgramBreakPoints schema)
        Output:
            response -> {code, msg}
        """
        req = common_msgs.ProgramBreakPoints()
        ParseDict(breakpoints, req)
        response = self.control.SetProgramBreakPoints(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_motion_j(self, request: dict):
        """
        Input:
            request -> dict (GetMotionJReq schema)
        Output:
            motion J data dict
        """
        req = control_msgs.GetMotionJReq()
        ParseDict(request, req)
        response = self.control.GetMotionJ(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_motion_l(self, request: dict):
        """
        Input:
            request -> dict (GetMotionLReq schema)
        Output:
            motion L data dict
        """
        req = control_msgs.GetMotionLReq()
        ParseDict(request, req)
        response = self.control.GetMotionL(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_motion_c(self, request: dict):
        """
        Input:
            request -> dict (GetMotionCReq schema)
        Output:
            motion C data dict
        """
        req = control_msgs.GetMotionCReq()
        ParseDict(request, req)
        response = self.control.GetMotionC(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def joint_to_tcp_transform(self, request: dict):
        """
        Input:
            request -> dict (JointToTcpTransformReq schema)
        Output:
            tpos -> float[6]
        """
        req = control_msgs.JointToTcpTransformReq()
        ParseDict(request, req)
        response = self.control.JointToTcpTransform(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movej_cond(self, request: dict):
        """
        Input:
            request -> dict (MoveJCondReq schema)
        Output:
            response -> {code, msg}
        """
        req = control_msgs.MoveJCondReq()
        ParseDict(request, req)
        response = self.control.MoveJCond(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def pause_motion(self, pause_category=PauseType.SMOOTH):
        """
        Input:
            pause_category -> PauseType (SMOOTH=0, IMMEDIATE=1)
        Output:
            response -> {code, msg}
        """
        response = self.control.PauseMotion(common_msgs.PauseCat(category=pause_category))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def program_step_into(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.ProgramStepInto(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def program_step_over(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.ProgramStepOver(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def program_step_out(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.ProgramStepOut(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def reset(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.Reset(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def search_program(self, prog_name: str = '', prog_idx: int = -1):
        """
        Input:
            prog_name -> string
            prog_idx -> int32
        Output:
            program search result dict
        """
        response = self.control.SearchProgram(control_msgs.Program(
            prog_name=prog_name,
            prog_idx=prog_idx
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def send_alarm(self, message: str):
        """
        Input:
            message -> string
        Output:
            response -> {code, msg}
        """
        response = self.control.SendAlarm(common_msgs.Message(content=message))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def send_annotation(self, message: str):
        """
        Input:
            message -> string
        Output:
            response -> {code, msg}
        """
        response = self.control.SendAnnotation(common_msgs.Message(content=message))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    

    ############################
    def set_do_config_list(self, do_config_list: dict):
        """
        Input:
            do_config_list -> {do_configs: [{state_code: int, state_name: string, onSignals: [{address, state}], offSignals: [{address, state}]}]}
        Output:
            response -> {code, msg}
        """
        do_list_request = config_msgs.DOConfigList()
        json_format.ParseDict(do_config_list, do_list_request)

        response = self.config.SetDOConfigList(do_list_request)

        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_do_config_list(self):
        """
        Output:
            do_configs -> [{state_code: int, state_name: string, onSignals: [{address, state}], offSignals: [{address, state}]}]
        """
        response = self.config.GetDOConfigList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def move_recover_joint(self, jtarget,
                           base_type=JointBaseType.ABSOLUTE) -> dict:
        """
        Input:
            jtarget -> float[] (joint angles in deg)
            base_type -> JointBaseType (ABSOLUTE=0, RELATIVE=1)
        Output:
            response -> {code, msg}
        """
        response = self.control.MoveRecoverJoint(
            control_msgs.TargetJ(j_target=list(jtarget), base_type=base_type)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_control_info(self):
        """
        Output:
            control info dict
        """
        response = self.control.GetControlInfo(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def check_aproach_retract_valid(self, tpos, init_jpos, pre_tpos, post_tpos):
        """
        Input:
            tpos -> float[6]
            init_jpos -> float[]
            pre_tpos -> float[6]
            post_tpos -> float[6]
        Output:
            valid -> bool
        """
        response = self.control.CheckAproachRetractValid(control_msgs.CheckAproachRetractValidReq(
            tpos=list(tpos),
            init_jpos=list(init_jpos),
            pre_tpos=list(pre_tpos),
            post_tpos=list(post_tpos)
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_pallet_point_list(self, tpos, jpos, pre_tpos, post_tpos, pallet_pattern, width, height):
        """
        Input:
            tpos -> float[6]
            jpos -> float[]
            pre_tpos -> float[6]
            post_tpos -> float[6]
            pallet_pattern -> int32
            width -> int32
            height -> int32
        Output:
            pallet point list dict
        """
        response = self.control.GetPalletPointList(control_msgs.GetPalletPointListReq(
            tpos=list(tpos),
            jpos=list(jpos),
            pre_tpos=list(pre_tpos),
            post_tpos=list(post_tpos),
            pallet_pattern=pallet_pattern,
            width=width,
            height=height
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    # def play_program_line(self, prog_name: str = '', prog_idx: int = -1):
    #     response = self.control.PlayProgramLine(control_msgs.Program(
    #         prog_name=prog_name,
    #         prog_idx=prog_idx
    #     ))
    #     return json_format.MessageToDict(response,
    #                                      including_default_value_fields=True,
    #                                      preserving_proto_field_name=True,
    #                                      use_integers_for_enums=True)

    def play_tuning_program(self, prog_name: str = '', prog_idx: int = -1,
                            tuning_space=common_msgs.TUNE_ALL, precision=common_msgs.HIGH_PRECISION,
                            vel_level_max=9):
        """
        Input:
            prog_name -> string
            prog_idx -> int32
            tuning_space -> int32 (TUNE_ALL, etc.)
            precision -> int32 (HIGH_PRECISION, etc.)
            vel_level_max -> int32
        Output:
            response -> {code, msg}
        """
        tuning_prog_dict = dict(
            program=dict(
                prog_name=prog_name,
                prog_idx=prog_idx),
            tuning_space=tuning_space,
            precision=precision,
            vel_level_max=vel_level_max
        )
        tuning_req = control_msgs.TuningProgram()

        # json_format.ParseDict(tuning_prog_dict, tuning_req)
        ParseDict(tuning_prog_dict, tuning_req)
        response = self.control.PlayTuningProgram(tuning_req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_di_config_list(self, di_config_list: dict):
        """
        Input:
            di_config_list -> {di_configs: [{function_code: int, function_name: string, triggerSignals: [{address, state}], successSignals: [{address, state}], failureSignals: [{address, state}]}]}
        Output:
            response -> {code, msg}
        """
        di_list_request = config_msgs.DIConfigList()
        # json_format.ParseDict(di_config_list, di_list_request)
        ParseDict(di_config_list, di_list_request)
        response = self.config.SetDIConfigList(di_list_request)

        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_di_config_list(self):
        """
        Output:
            di_configs -> [{function_code: int, function_name: string, triggerSignals: [{address, state}], successSignals: [{address, state}], failureSignals: [{address, state}]}]
        """
        response = self.config.GetDIConfigList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_ft_sensor_config(self,
                          dev_type, com_type, ip_address,
                               ft_frame_translation_offset_x=0.0,
                               ft_frame_translation_offset_y=0.0,
                               ft_frame_translation_offset_z=0.0,
                               ft_frame_rotation_offset_r=0.0,
                               ft_frame_rotation_offset_p=0.0,
                               ft_frame_rotation_offset_y=0.0):
        """
        Input:
            dev_type -> int32
            com_type -> int32
            ip_address -> string
            ft_frame_translation_offset_x -> float
            ft_frame_translation_offset_y -> float
            ft_frame_translation_offset_z -> float
            ft_frame_rotation_offset_r -> float
            ft_frame_rotation_offset_p -> float
            ft_frame_rotation_offset_y -> float
        Output:
            response -> {code, msg}
        """
        response = self.config.SetFTSensorConfig(config_msgs.FTSensorDevice(
            dev_type=dev_type, com_type=com_type,ip_address=ip_address,
            ft_frame_translation_offset_x=ft_frame_translation_offset_x,
            ft_frame_translation_offset_y=ft_frame_translation_offset_y,
            ft_frame_translation_offset_z=ft_frame_translation_offset_z,
            ft_frame_rotation_offset_r=ft_frame_rotation_offset_r,
            ft_frame_rotation_offset_p=ft_frame_rotation_offset_p,
            ft_frame_rotation_offset_y=ft_frame_rotation_offset_y))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ft_sensor_config(self):
        """
        Output:
            dev_type -> int32
            com_type -> int32
            ip_address -> string
            ft_frame_translation_offset_x -> float
            ft_frame_translation_offset_y -> float
            ft_frame_translation_offset_z -> float
            ft_frame_rotation_offset_r -> float
            ft_frame_rotation_offset_p -> float
            ft_frame_rotation_offset_y -> float
        """
        response = self.config.GetFTSensorConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_auto_servo_off(self, enable: bool, time: float):
        """
        Input:
            enable -> bool
            time -> float (seconds)
        Output:
            response -> {code, msg}
        """
        response = self.config.SetAutoServoOff(config_msgs.AutoServoOffConfig(
            enable=enable, time=time
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_auto_servo_off(self):
        """
        Output:
            enable -> bool
            time -> float
        """
        response = self.config.GetAutoServoOff(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_safety_stop_config(self, jpos_limit_stop_cat=StopCategory.CAT0,
                               jvel_limit_stop_cat=StopCategory.CAT0,
                               jtau_limit_stop_cat=StopCategory.CAT0,
                               tvel_limit_stop_cat=StopCategory.CAT0,
                               tforce_limit_stop_cat=StopCategory.CAT0,
                               power_limit_stop_cat=StopCategory.CAT0,
                               safegd_stop_cat=None,
                               safegd_type=None):
        """
        Input:
            jpos_limit_stop_cat -> StopCategory (CAT0=0, CAT1=1, CAT2=2)
            jvel_limit_stop_cat -> StopCategory
            jtau_limit_stop_cat -> StopCategory
            tvel_limit_stop_cat -> StopCategory
            tforce_limit_stop_cat -> StopCategory
            power_limit_stop_cat -> StopCategory
            safegd_stop_cat -> StopCategory[] (optional)
            safegd_type -> int32[] (optional)
        Output:
            response -> {code, msg}
        """
        response = self.config.SetSafetyStopConfig(config_msgs.SafetyStopConfig(
            joint_position_limit_stop_cat=jpos_limit_stop_cat,
            joint_speed_limit_stop_cat=jvel_limit_stop_cat,
            joint_torque_limit_stop_cat=jtau_limit_stop_cat,
            tcp_speed_limit_stop_cat=tvel_limit_stop_cat,
            tcp_force_limit_stop_cat=tforce_limit_stop_cat,
            power_limit_stop_cat=power_limit_stop_cat,
            safegd_stop_cat=(safegd_stop_cat or []),
            safegd_type=(safegd_type or [])
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_safety_stop_config(self):
        """
        Output:
            joint_position_limit_stop_cat -> int32
            joint_speed_limit_stop_cat -> int32
            joint_torque_limit_stop_cat -> int32
            tcp_speed_limit_stop_cat -> int32
            tcp_force_limit_stop_cat -> int32
            power_limit_stop_cat -> int32
            safegd_stop_cat -> int32[]
            safegd_type -> int32[]
        """
        response = self.config.GetSafetyStopConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_reduced_ratio(self):
        """
        Output:
            ratio -> float
        """
        response = self.config.GetReducedRatio(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_reduced_speed(self):
        """
        Output:
            speed -> float
        """
        response = self.config.GetReducedSpeed(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_reduced_speed(self, speed):
        """
        Input:
            speed -> float
        Output:
            response -> {code, msg}
        """
        response = self.config.SetReducedSpeed(config_msgs.SetReducedSpeedReq(speed=speed))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_teleop_params(self, smooth_factor, cutoff_freq, error_gain):
        """
        Input:
            smooth_factor -> float
            cutoff_freq -> float
            error_gain -> float
        Output:
            response -> {code, msg}
        """
        response = self.config.SetTeleOpParams(
            config_msgs.TeleOpParams(smooth_factor=smooth_factor,
                                     cutoff_freq=cutoff_freq,
                                     error_gain=error_gain))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_teleop_params(self):
        """
        Output:
            smooth_factor -> float
            cutoff_freq -> float
            error_gain -> float
        """
        response = self.config.GetTeleOpParams(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_kinematics_params(self):
        """
        Output:
            kinematics params dict
        """
        response = self.config.GetKinematicsParams(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_io_data(self):
        """
        Output:
            di -> DigitalSignal[]
            do -> DigitalSignal[]
            ai -> AnalogSignal[]
            ao -> AnalogSignal[]
            end_di -> EndtoolSignal[]
            end_do -> EndtoolSignal[]
            end_ai -> AnalogSignal[]
            end_ao -> AnalogSignal[]
            response -> {code, msg}
        """
        response = self.rtde.GetIOData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def ping_from_conty(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.PingFromConty(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                    including_default_value_fields=True,
                                    preserving_proto_field_name=True,
                                    use_integers_for_enums=True)

    def load_reference_frame(self):
        """
        Output:
            ref_frames -> [{name: string, tpos: float[], ...}]
            default_name -> string
        """
        response = self.config.GetRefFrameList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                    including_default_value_fields=True,
                                    preserving_proto_field_name=True,
                                    use_integers_for_enums=True)
        
    def save_reference_frame(self, frames, default_name):
        """
        Input:
            frames -> [{name: string, tpos: float[], tpos0~2: float[], jpos0~2: float[]}]
            default_name -> string
        Output:
            response -> {code, msg}
        """
        request = config_msgs.RefFrameList(
            ref_frames=[
                common_msgs.NamedReferencePosition(
                    name=frame['name'],
                    tpos=frame.get('tpos', []),
                    tpos0=frame.get('tpos0', []),
                    tpos1=frame.get('tpos1', []),
                    tpos2=frame.get('tpos2', []),
                    jpos0=frame.get('jpos0', []),
                    jpos1=frame.get('jpos1', []),
                    jpos2=frame.get('jpos2', [])
                ) for frame in frames
            ],
            default_name=default_name
        )

        response = self.config.SetRefFrameList(request)
        return json_format.MessageToDict(response,
                                    including_default_value_fields=True,
                                    preserving_proto_field_name=True,
                                    use_integers_for_enums=True)

    def get_ft_zero(self):
        """
        Output:
            response -> {code, msg}
        """
        response = self.control.FTZero(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
        
    def get_inference_data(self):
        """
        Output:
            infdata0~5 -> float[6] each
        """
        response = self.control.GetControlInferenceData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_inference_data(self, infdata0, infdata1, infdata2, infdata3, infdata4, infdata5):
        """
        Input:
            infdata0~5 -> float[6] each
        Output:
            response -> {code, msg}
        """

        response = self.control.SetControlInferenceData(control_msgs.ControlInferenceDataSet(
            infdata0=infdata0,
            infdata1=infdata1,
            infdata2=infdata2,
            infdata3=infdata3,
            infdata4=infdata4,
            infdata5=infdata5
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_inference_data(self, *args):        
        """
        Input:
            *args -> up to 6 float[6] arrays
        Output:
            response -> {code, msg}
        """

        infdata = [[0.0] * 6 for _ in range(6)]

        for i, value in enumerate(args):
            if i < 6:
                if isinstance(value, list) and len(value) == 6:
                    infdata[i] = value
                else:
                    print(f"Debug 2: Argument {i} ignored - Expected a list of size 6 but got {value}.")
            else:
                print(f"Debug 2: Argument {i} = {value} ignored (index exceeds 5).")

        response = self.control.SetControlInferenceData(control_msgs.ControlInferenceDataSet(
            infdata0=infdata[0],
            infdata1=infdata[1],
            infdata2=infdata[2],
            infdata3=infdata[3],
            infdata4=infdata[4],
            infdata5=infdata[5]
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
        