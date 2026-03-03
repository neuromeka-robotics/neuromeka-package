import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *

from google.protobuf import json_format


class DeviceMixin:
    """Mixin for Device channel (port 20002/30002) methods."""

    def commit_violation(self, violation: dict):
        req = device_msgs.ViolationRequest()
        json_format.ParseDict(violation, req)
        response = self.device.CommitViolation(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_rt_task_times(self):
        response = self.device.GetRTTaskTimes(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # IO board and Endtool port interfaces
    ############################
    def get_di(self):
        """
        address = uint32
        state = DigitalState
        """
        response = self.device.GetDI(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_do(self):
        """
        signals = index
        address = uint32
        state = DigitalState
        """
        response = self.device.GetDO(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_do(self, do_signal_list: list):
        """
        do_list = [(int_addr1, True/False), (int_addr1, True/False), ...]
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

    def get_ai(self) -> list:
        """
        address = uint32
        voltage = int32
        """
        response = self.device.GetAI(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ao(self) -> list:
        """
        address = uint32
        voltage = int32
        """
        response = self.device.GetAO(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_ao(self, ao_signal_list: list):
        response = self.device.SetAO(device_msgs.AnalogList(
            signals=self.__to_analog_request_list__(ao_signal_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_di(self) -> list:
        """
        state = EndtoolState
        port = char value [A,B,C]
        """
        response = self.device.GetEndDI(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_do(self) -> list:
        """
        state = EndtoolState
        port = char value [A,B,C]
        """
        response = self.device.GetEndDO(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_endtool_do(self, end_do_signal_list: list):
        response = self.device.SetEndDO(device_msgs.EndtoolSignalList(
            signals=self.__to_endtool_signal_list__(end_do_signal_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_ai(self) -> list:
        """
        address = uint32
        voltage = int32
        """
        response = self.device.GetEndAI(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_ao(self) -> list:
        """
        address = uint32
        voltage = int32
        """
        response = self.device.GetEndAO(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_endtool_ao(self, end_ao_signal_list: list):
        response = self.device.SetEndAO(device_msgs.AnalogList(
            signals=self.__to_analog_request_list__(end_ao_signal_list),
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_rs485_rx(self) -> dict:
        response = self.device.GetEndRS485Rx(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_rs485_rx_for(self, index: int) -> dict:
        response = self.device.GetEndRS485RxFor(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_rs485_tx(self) -> dict:
        response = self.device.GetEndRS485Tx(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_endtool_rs485_tx_for(self, index: int) -> dict:
        response = self.device.GetEndRS485TxFor(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_endtool_rs485_rx(self, word1: int, word2: int):
        response = self.device.SetEndRS485Rx(common_msgs.EndtoolRS485Rx(
            word1=word1, word2=word2
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_endtool_led_dim(self, led_dim, tool_index=0):
        response = self.device.SetEndLedDim(device_msgs.EndLedDim(led_dim=led_dim, tool_index=tool_index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    
    def execute_tool(self, name: str):
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
        response = self.device.GetBrakeControlStyle(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_device_info(self):
        """
        Device Info:
            num_joints   -> uint32
            robot_serial   -> string
            io_board_fw_ver  -> string
            core_board_fw_vers  -> string[6]
            endtool_board_fw_ver  -> string
            endtool_port_type  -> EndToolPortType
            teleop_loaded -> bool
            calibrated -> bool
        """
        response = self.device.GetDeviceInfo(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Conveyor
    ############################
    def get_conveyor(self):
        response = self.device.GetConveyor(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_name(self, name: str):
        response = self.device.SetConveyorName(common_msgs.Name(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_by_name(self, name: str):
        response = self.device.SetConveyorByName(common_msgs.Name(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_encoder(self, encoder_type, channel1: int, channel2: int, sample_num: int,
                           mm_per_tick: float, vel_const_mmps: float, reversed: bool):
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
        response = self.device.SetConveyorTrigger(
            device_msgs.Trigger(type=trigger_type, channel=channel, detect_rise=detect_rise)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_offset(self, offset_mm):
        response = self.device.SetConveyorOffset(common_msgs.Float(value=offset_mm))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_locked_joint(self, index: int):
        response = self.device.SetConveyorLockedJoint(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_arm_index(self, index: int):
        response = self.device.SetConveyorArmIndex(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_tool_link(self, index: int):
        response = self.device.SetConveyorToolLink(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_starting_pose(self, jpos, tpos):
        response = self.device.SetConveyorStartingPose(
            common_msgs.PosePair(q=jpos, p=tpos)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_terminal_pose(self, jpos, tpos):
        response = self.device.SetConveyorTerminalPose(
            common_msgs.PosePair(q=jpos, p=tpos)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_conveyor_state(self):
        response = self.device.GetConveyorState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Sander
    ############################
    def set_sander_command(self, sander_type, ip: str, speed: float, state: bool, tool_index=0):
        response = self.device.SetSanderCommand(
            device_msgs.SanderCommand(type=sander_type, ip=ip, speed=speed, state=state, tool_index=tool_index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_sander_command(self):
        """
        SanderCommand:
            type   -> SanderType
            ip   -> string
            speed  -> float
            state  -> bool
        """
        response = self.device.GetSanderCommand(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_sander_command_for(self, index: int):
        response = self.device.GetSanderCommandFor(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Photoneo / Vision
    ############################
    def add_photoneo_calib_point(self, vision_name, px, py, pz, arm_index=0):
        response = self.device.AddPhotoneoCalibPoint(
            device_msgs.AddPhotoneoCalibPointReq(vision_name=vision_name, px=px, py=py, pz=pz, arm_index=arm_index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_photoneo_detection(self, vision_server, object, frame_type):
        response = self.device.GetPhotoneoDetection(
            device_msgs.VisionRequest(vision_server=vision_server, object=object, frame_type=frame_type))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_photoneo_retrieval(self, vision_server, object, frame_type):
        response = self.device.GetPhotoneoRetrieval(
            device_msgs.VisionRequest(vision_server=vision_server, object=object, frame_type=frame_type))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # FT Sensor
    ############################
    def get_ft_sensor_data(self):
        """
        FT Sensor Data:
        """
        response = self.device.GetFTSensorData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ft_sensor_data_for(self, index: int):
        response = self.device.GetFTSensorDataFor(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_load_factors(self):
        """
        Device Info:
            num_joints   -> uint32
            robot_serial   -> string
            io_board_fw_ver  -> string
            core_board_fw_vers  -> string[6]
            endtool_board_fw_ver  -> string
            endtool_port_type  -> EndToolPortType
            response  -> {code: int64, msg: string}
        """
        response = self.device.GetLoadFactors(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Safety
    ############################
    def set_auto_mode(self, on: bool):
        response = self.device.SetAutoMode(device_msgs.SetAutoModeReq(on=on))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def check_auto_mode(self):
        response = self.device.CheckAutoMode(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def check_reduced_mode(self):
        response = self.device.CheckReducedMode(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_safety_function_state(self):
        response = self.device.GetSafetyFunctionState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def request_safety_function(self, id, state):
        response = self.device.RequestSafetyFunction(
            device_msgs.SafetyFunctionState(id = id, state = state))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_safety_control_data(self):
        response = self.device.GetSafetyControlData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Gripper
    ############################
    def get_gripper_data(self) -> list:
        response = self.device.GetGripperData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_gripper_command(self,
                            command,
                            gripper_type,
                            pvt_data,
                            tool_index=0,
                            gripper_modbus_use: bool = False,
                            gripper_modbus_server_name: str = ""):
        # gripper_pvt_data is a repeated int32 in proto; normalize common input shapes.
        if pvt_data is None:
            pvt_data = []
        elif not isinstance(pvt_data, (list, tuple)):
            pvt_data = [pvt_data]
        pvt_data = [int(x) for x in pvt_data]

        response = self.device.SetGripperCommand(device_msgs.GripperCommand(
            gripper_command=command,
            gripper_type=gripper_type,
            gripper_pvt_data=pvt_data,
            gripper_modbus_use=gripper_modbus_use,
            gripper_modbus_server_name=gripper_modbus_server_name,
            tool_index=tool_index,
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Motor / Brake / Servo
    ############################
    def set_brakes(self, brake_state_list: list):
        """
        brake_state_list -> bool[6]
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
        enable -> bool
        """
        response = self.device.SetServoAll(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_servo(self, index, enable=True):
        """
        index -> int
        enable -> bool
        """
        response = self.device.SetServo(device_msgs.Servo(index=index, enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Socket Command
    ############################
    def socket_cmd_set_config(self, remote_ip: str, remote_port: int,
                              auto_reconnect: bool = False, rx_buffer_size: int = 1024):
        """
        Socket Command - Set Configuration:
            remote_ip -> string
            remote_port -> uint32
            auto_reconnect -> bool
            rx_buffer_size -> uint32
        """
        response = self.device.SocketCmdSetConfig(device_msgs.SocketCommandConfig(
            remote_ip=remote_ip,
            remote_port=remote_port,
            auto_reconnect=auto_reconnect,
            rx_buffer_size=rx_buffer_size
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def socket_cmd_get_config(self):
        """
        Socket Command - Get Configuration:
            remote_ip -> string
            remote_port -> uint32
            auto_reconnect -> bool
            rx_buffer_size -> uint32
        """
        response = self.device.SocketCmdGetConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def socket_cmd_start(self):
        """
        Socket Command - Start connection.
        Returns SocketCommandStatus:
            connected -> bool
            last_error -> string
        """
        response = self.device.SocketCmdStart(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def socket_cmd_stop(self):
        """
        Socket Command - Stop connection.
        Returns SocketCommandStatus:
            connected -> bool
            last_error -> string
        """
        response = self.device.SocketCmdStop(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def socket_cmd_send_data(self, data: bytes):
        """
        Socket Command - Send data.
            data -> bytes
        Returns SocketCommandStatus:
            connected -> bool
            last_error -> string
        """
        response = self.device.SocketCmdSendData(device_msgs.SocketPayload(data=data))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def socket_cmd_get_latest_data(self):
        """
        Socket Command - Get latest received data.
        Returns SocketPayload:
            data -> bytes
        """
        response = self.device.SocketCmdGetLatestData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Inspire Hand RH56
    ############################
    def set_inspire_hand_command(self, tool_index: int = 0, slave_id: int = 1,
                                 speeds: list = None, forces: list = None,
                                 angles: list = None,
                                 request_feedback: bool = False,
                                 apply_speed: bool = False,
                                 apply_force: bool = False,
                                 apply_angle: bool = False):
        """
        Inspire Hand RH56 Command:
            tool_index -> int
            slave_id -> uint32 (default 1)
            speeds -> int[6] (optional)
            forces -> int[6] (optional)
            angles -> int[6] (optional, -1 -> no move)
            request_feedback -> bool
            apply_speed -> bool
            apply_force -> bool
            apply_angle -> bool
        """
        response = self.device.SetInspireHandCommand(device_msgs.InspireHandCommand(
            tool_index=tool_index,
            slave_id=slave_id,
            speeds=speeds or [],
            forces=forces or [],
            angles=angles or [],
            request_feedback=request_feedback,
            apply_speed=apply_speed,
            apply_force=apply_force,
            apply_angle=apply_angle
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_inspire_hand_state(self, tool_index: int = 0):
        """
        Inspire Hand RH56 State:
            tool_index -> int
            slave_id -> uint32
            angles -> int[6] (actual angles)
            comm_ok -> bool
            crc_error_count -> uint32
            msg -> string
        """
        response = self.device.GetInspireHandState(common_msgs.Int(value=tool_index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
