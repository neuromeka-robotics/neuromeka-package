import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *

from google.protobuf import json_format
from ._helpers import _message_to_dict


class RTDEChannelAPI:
    """ChannelAPI for RTDE channel (port 20004/30004) methods."""

    def get_robot_data(self):
        """
        Control Data:
            running_hours   -> uint32
            running_mins   -> uint32
            running_secs  -> uint32
            op_state  -> OpState
            sim_mode  -> bool
            q  -> float[6]
            qdot  -> float[6]
            p  -> float[6]
            pdot  -> float[6]
            ref_frame  -> float[6]
            tool_frame  -> float[6]
            ref_links  -> int[]
            response  -> Response
        """
        response = self.rtde.GetControlData(common_msgs.Empty())
        return _message_to_dict(response)

    def get_control_state(self):
        """
        Control Data:
            q  -> float[]
            qdot  -> float[]
            qddot  -> float[]
            qdes  -> float[]
            qdotdes  -> float[]
            qddotdes  -> float[]
            p  -> float[]
            pdot  -> float[]
            pddot  -> float[]
            pdes  -> float[]
            pdotdes  -> float[]
            pddotdes  -> float[]
            tau  -> float[]
            tau_act  -> float[]
            tau_ext  -> float[]
            tau_jts  -> float[]
        """
        response = self.rtde.GetControlState(common_msgs.Empty())
        return _message_to_dict(response)

    def get_motion_data(self):
        """
        Motion Data:
            traj_state   -> TrajState
            traj_progress   -> int32
            is_in_motion  -> bool
            is_target_reached  -> bool
            is_pausing  -> bool
            is_stopping  -> bool
            has_motion  -> bool
            speed_ratio  -> int32
            motion_id  -> int32
            remain_distance  -> float
            motion_queue_size  -> uint32
            cur_traj_progress  -> int32
        """
        response = self.rtde.GetMotionData(common_msgs.Empty())
        return _message_to_dict(response)

    def get_servo_data(self):
        """
        Servo Data:
            status_codes   -> string[]
            temperatures   -> float[]
            voltages  -> float[]
            currents  -> float[]
            servo_actives  -> bool[]
            brake_actives  -> bool[]
        """
        response = self.rtde.GetServoData(common_msgs.Empty())
        return _message_to_dict(response)

    def get_collision_model_state(self):
        response = self.rtde.GetCollisionModelState(common_msgs.Empty())
        return _message_to_dict(response)

    def get_reserved_data(self):
        response = self.rtde.GetReservedData(common_msgs.Empty())
        return _message_to_dict(response)

    def get_violation_data(self):
        """
        Violation Data:
            violation_code   -> uint64
            j_index   -> uint32
            i_args  -> int32[]
            f_args  -> float[]
            violation_str  -> string
        """
        response = self.rtde.GetViolationData(common_msgs.Empty())
        return _message_to_dict(response)

    def get_violation_message_queue(self):
        """
        Violation Data:
            violation_queue   -> ViolationData[]
        """
        response = self.rtde.GetViolationMessageQueue(common_msgs.Empty())
        return _message_to_dict(response)

    def get_program_data(self):
        """
        Program Data:
            program_state   -> ProgramState
            cmd_id   -> int32
            sub_cmd_id  -> int32
            running_hours  -> int32
            running_mins  -> int32
            running_secs  -> int32
            program_name  -> string
            program_alarm  -> string
            program_annotation  -> string
            speed_ratio -> int32
        """
        response = self.rtde.GetProgramData(common_msgs.Empty())
        return _message_to_dict(response)

    def get_stop_state(self):
        """
        Program Data:
            category   -> StopCategory
        """
        response = self.rtde.GetStopState(common_msgs.Empty())
        return _message_to_dict(response)

    def get_io_data(self):
        """
        IO Data:
            di   -> DigitalSignal[]
            do   -> DigitalSignal[]
            ai  -> AnalogSignal[]
            ao  -> AnalogSignal[]
            end_di  -> EndtoolSignal[]
            end_do  -> EndtoolSignal[]
            end_ai  -> AnalogSignal[]
            end_ao  -> AnalogSignal[]
            response  -> Response
        """
        response = self.rtde.GetIOData(common_msgs.Empty())
        return _message_to_dict(response)

    def test_function(self, request):
        return _message_to_dict(self.rtde.TestFunction(request))
