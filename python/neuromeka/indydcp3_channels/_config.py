import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *

from neuromeka.enums import *

from google.protobuf import json_format
from google.protobuf.json_format import ParseDict


class ConfigMixin:
    """Mixin for Config channel (port 20003/30003) methods."""

    def set_speed_ratio(self, speed_ratio: int):
        """
        Speed Ratio
            ratio -> uint32 {0 ~ 100}
        """
        response = self.config.SetSpeedRatio(config_msgs.Ratio(
            ratio=speed_ratio
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_pack_pos(self):
        """
        Joint Pack Position
            jpos -> double[]
        """
        response = self.config.GetPackPosition(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_path_config(self):
        response = self.config.GetPathConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_locked_joint(self, index: int):
        response = self.config.SetLockedJoint(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_link(self, index: int):
        response = self.config.SetToolLink(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_speed_ratio(self):
        response = self.config.GetSpeedRatio(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_list(self, tool_list: dict):
        req = config_msgs.ToolList()
        ParseDict(tool_list, req)
        response = self.config.SetToolList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_list(self):
        response = self.config.GetToolList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_vision_server_list(self):
        response = self.config.GetVisionServerList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_vision_server_list(self, vision_server_list: dict):
        req = config_msgs.VisionServerList()
        ParseDict(vision_server_list, req)
        response = self.config.SetVisionServerList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_modbus_server_list(self):
        response = self.config.GetModbusServerList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_modbus_server_list(self, modbus_server_list: dict):
        req = config_msgs.ModbusServerList()
        ParseDict(modbus_server_list, req)
        response = self.config.SetModbusServerList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_conveyor_list(self):
        response = self.config.GetConveyorList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_conveyor_list(self, conveyor_list: dict):
        req = config_msgs.ConveyorList()
        ParseDict(conveyor_list, req)
        response = self.config.SetConveyorList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_compliance_control_joint_gain(self, gains: dict):
        req = config_msgs.ComplianceGainSet()
        ParseDict(gains, req)
        response = self.config.SetComplianceControlJointGain(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_compliance_control_joint_gain(self):
        response = self.config.GetComplianceControlJointGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_frame_list(self):
        response = self.config.GetToolFrameList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_frame_list(self, tool_frame_list: dict):
        req = config_msgs.ToolFrameList()
        ParseDict(tool_frame_list, req)
        response = self.config.SetToolFrameList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ref_frame_list(self):
        response = self.config.GetRefFrameList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_ref_frame_list(self, ref_frame_list: dict):
        req = config_msgs.RefFrameList()
        ParseDict(ref_frame_list, req)
        response = self.config.SetRefFrameList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_custom_pos_list(self):
        response = self.config.GetCustomPosList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_custom_pos_list(self, custom_pos_list: dict):
        req = config_msgs.CustomPosList()
        ParseDict(custom_pos_list, req)
        response = self.config.SetCustomPosList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_shape_list(self, tool_shape_list: dict):
        req = config_msgs.ToolShapeList()
        ParseDict(tool_shape_list, req)
        response = self.config.SetToolShapeList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_shape_list(self):
        response = self.config.GetToolShapeList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_environment_list(self, environment_list: dict):
        req = config_msgs.EnvironmentList()
        ParseDict(environment_list, req)
        response = self.config.SetEnvironmentList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_environment_list(self):
        response = self.config.GetEnvironmentList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_default_coll_sens_param(self):
        response = self.config.GetDefaultCollSensParam(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_sensorless_params(self, params: dict):
        req = config_msgs.SensorlessParams()
        ParseDict(params, req)
        response = self.config.SetSensorlessParams(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_sensorless_params(self):
        response = self.config.GetSensorlessParams(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_on_start_program_config(self, config: dict):
        req = config_msgs.OnStartProgramConfig()
        ParseDict(config, req)
        response = self.config.SetOnStartProgramConfig(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_on_start_program_config(self):
        response = self.config.GetOnStartProgramConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_simple_coll_threshold(self):
        response = self.config.SetSimpleCollThreshold(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_collison_model_margin(self):
        response = self.config.GetCollisonModelMargin(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_collison_model_margin(self, collision_margin: float, recover_margin: float):
        response = self.config.SetCollisonModelMargin(
            config_msgs.CollisionModelMargin(collision_margin=collision_margin,
                                             recover_margin=recover_margin)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Home / Frames
    ############################
    def get_home_pos(self):
        """
        Joint Home Position
            jpos -> double[]
        """
        response = self.config.GetHomePosition(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_home_pos(self, home_jpos: list):
        """
        Joint Home Position
            jpos -> double[]
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
        Reference frame
            fpos -> float[6]
        """
        response = self.config.GetRefFrame(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ref_frame_for(self, index: int):
        """
        Reference frame at specific index
            index -> int
            returns Frame
        """
        response = self.config.GetRefFrameFor(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_ref_frame(self, fpos: list, arm_index: int = 0, link_index: int = 0):
        """
        Ref Frame
            fpos -> float[6]
        """
        response = self.config.SetRefFrame(config_msgs.Frame(
            fpos=list(fpos), arm_index=arm_index, link_index=link_index
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_ref_frame_planar(self, fpos0: list, fpos1: list, fpos2: list, arm_index: int = 0):
        """
        Ref Frame
            fpos -> float[6]
        """
        response = self.config.SetRefFramePlanar(config_msgs.PlanarFrame(
            fpos0=list(fpos0), fpos1=list(fpos1), fpos2=list(fpos2), arm_index=arm_index
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_frame(self, fpos: list, arm_index: int = 0, link_index: int = 0):
        """
        Tool Frame
            fpos -> float[6]
        """
        response = self.config.SetToolFrame(config_msgs.Frame(
            fpos=list(fpos), arm_index=arm_index, link_index=link_index
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Friction Compensation Config
    ############################
    def get_friction_comp(self):
        """
        Friction Compensation Set:
            joint_idx   -> uint32
            control_comp_enable   -> bool
            control_comp_levels   -> int32[6]
            teaching_comp_enable   -> bool
            teaching_comp_levels   -> int32[6]
        """
        response = self.config.GetFrictionComp(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_friction_comp(self, control_comp: bool, control_comp_levels: list,
                          dt_comp: bool, dt_comp_levels: list):
        """
        Friction Compensation Set:
            joint_idx   -> uint32
            control_comp_enable   -> bool
            control_comp_levels   -> int32[6]
            teaching_comp_enable   -> bool
            teaching_comp_levels   -> int32[6]
        """
        response = self.config.SetFrictionComp(config_msgs.FrictionCompSet(
            control_comp_enable=control_comp, control_comp_levels=list(control_comp_levels),
            teaching_comp_enable=dt_comp, teaching_comp_levels=list(dt_comp_levels)
        ))

        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Mounting
    ############################
    def set_mount_pos(self, rot_y=0.0, rot_z=0.0):
        """
        Mounting Angles:
            rot_y   -> float
            rot_z   -> float
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
        Mounting Angles:
            rot_y   -> float
            rot_z   -> float
        """
        response = self.config.GetMountPos(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Tool Properties
    ############################
    def get_tool_property(self):
        """
        Tool Properties:
            mass -> float
            center_of_mass -> float[3]
            inertia -> float[6]
            arm_index -> int
            link_index -> int (deprecated)
        """
        response = self.config.GetToolProperty(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_property_at(self, index: int):
        """
        Tool Properties at index:
            index -> int
            returns ToolProperties (mass, center_of_mass[3], inertia[6], arm_index, link_index(deprecated))
        """
        response = self.config.GetToolPropertyAt(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tool_property(self, mass: float, center_of_mass: list, inertia: list, arm_index: int = 0):
        """
        Tool Properties:
            mass   -> float
            center_of_mass   -> float[3]
            inertia   -> float[6]
            arm_index -> int
        """
        response = self.config.SetToolProperty(config_msgs.ToolProperties(
            mass=mass, center_of_mass=list(center_of_mass),
            inertia=list(inertia), arm_index=arm_index
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Collision
    ############################
    def get_coll_sens_level(self):
        """
        Collision Sensitivity Level:
            level -> uint32
        """
        response = self.config.GetCollSensLevel(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_coll_sens_level(self, level: int):
        """
        Collision Sensitivity Level:
            level -> uint32
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
        Collision Params:
            j_torque_bases                  -> double[6]
            j_torque_tangents               -> double[6]
            t_torque_bases                  -> double[6]
            t_torque_tangents               -> double[6]
            error_bases                     -> double[6]
            error_tangents                  -> double[6]
            t_constvel_torque_bases         -> double[6]
            t_constvel_torque_tangents      -> double[6]
            t_conveyor_torque_bases         -> double[6]
            t_conveyor_torque_tangents      -> double[6]
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
        Collision Params:
            j_torque_bases                  -> double[6]
            j_torque_tangents               -> double[6]
            t_torque_bases                  -> double[6]
            t_torque_tangents               -> double[6]
            error_bases                     -> double[6]
            error_tangents                  -> double[6]
            t_constvel_torque_bases         -> double[6]
            t_constvel_torque_tangents      -> double[6]
            t_conveyor_torque_bases         -> double[6]
            t_conveyor_torque_tangents      -> double[6]
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
        Collision Policy:
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
        Collision Policies:
            policy -> uint32
            sleep_time -> float
            gravity_time -> float
        """
        CollisionPolicyType.NONE
        response = self.config.SetCollPolicy(config_msgs.CollisionPolicy(
            policy=policy, sleep_time=sleep_time, gravity_time=gravity_time
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Safety Limits
    ############################
    def get_safety_limits(self):
        """
        Safety Limits:
            power_limit             -> float
            power_limit_ratio       -> float
            tcp_force_limit         -> float
            tcp_force_limit_ratio   -> float
            tcp_speed_limit         -> float
            tcp_speed_limit_ratio   -> float
            joint_upper_limits   -> float[]
            joint_lower_limits   -> float[]
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
        Safety Limits:
            power_limit             -> float
            power_limit_ratio       -> float
            tcp_force_limit         -> float
            tcp_force_limit_ratio   -> float
            tcp_speed_limit         -> float
            tcp_speed_limit_ratio   -> float
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
    # Control Gains
    ############################
    def get_custom_control_gain(self):
        """
        Custom Control Gain
            gain0   -> float[6]
            gain1   -> float[6]
            gain2   -> float[6]
            gain3   -> float[6]
            gain4   -> float[6]
            gain5   -> float[6]
            gain6   -> float[6]
            gain7   -> float[6]
            gain8   -> float[6]
            gain9   -> float[6]
        """
        response = self.config.GetCustomControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    
    def set_custom_control_gain(self, gain0=None, gain1=None, gain2=None, gain3=None, gain4=None, 
                                gain5=None, gain6=None, gain7=None, gain8=None, gain9=None):
        """
        Set custom control gains with up to 10 gain arrays.
        Args:
            gain0, gain1, ..., gain9: Up to 10 lists of gain values. Each gain should be a list of floats.
                                    If a gain is None, it will be replaced with a default list [0, 0, 0, 0, 0, 0].
        """
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
        Joint Control Gains:
            kp   -> float[6]
            kv   -> float[6]
            kl2  -> float[6]
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
        Joint Control Gains:
            kp   -> float[6]
            kv   -> float[6]
            kl2  -> float[6]
        """
        response = self.config.GetJointControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_task_control_gain(self, kp, kv, kl2):
        """
        Task Control Gains:
            kp   -> float[6]
            kv   -> float[6]
            kl2  -> float[6]
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
        Task Control Gains:
            kp   -> float[6]
            kv   -> float[6]
            kl2  -> float[6]
        """
        response = self.config.GetTaskControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_impedance_control_gain(self, mass, damping, stiffness, kl2):
        """
        Impedance Control Gains:
            mass   -> float[6]
            damping   -> float[6]
            stiffness   -> float[6]
            kl2  -> float[6]
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
        Impedance Control Gains:
            mass   -> float[6]
            damping   -> float[6]
            stiffness   -> float[6]
            kl2  -> float[6]
        """
        response = self.config.GetImpedanceControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_force_control_gain(self, kp, kv, kl2, mass, damping, stiffness, kpf, kif):
        """
        Impedance Control Gains:
            mass   -> float[6]
            damping   -> float[6]
            stiffness   -> float[6]
            kl2  -> float[6]
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
        Impedance Control Gains:
            mass   -> float[6]
            damping   -> float[6]
            stiffness   -> float[6]
            kl2  -> float[6]
        """
        response = self.config.GetForceControlGain(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # IO Config
    ############################
    def set_do_config_list(self, do_config_list: dict):
        """
        DO Configuration List
            {
                'do_configs': [
                    {
                        'state_code': 2,
                        'state_name': "name",
                        'onSignals': [{'address': 1, 'state': 1, 'do_mode': 0, ...}],
                        'offSignals': [{'address': 1, 'state': 1, 'do_mode': 0, ...}]
                    }
                ]
            }
        Note: onSignals/offSignals use DOChannelMode format.
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
        DO Configuration List
            {
                'do_configs': [
                    {
                        'state_code': 2,
                        'state_name': "name",
                        'onSignals': [{'address': 1, 'state': 1, 'do_mode': 0, ...}],
                        'offSignals': [{'address': 1, 'state': 1, 'do_mode': 0, ...}]
                    }
                ]
            }
        Note: onSignals/offSignals use DOChannelMode format.
        """
        response = self.config.GetDOConfigList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_di_config_list(self, di_config_list: dict):
        """
        DI Configuration List
            {
                'di_configs': [
                    {
                        'function_code': 2,
                        'function_name': "name",
                        'triggerSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                        'successSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                        'failureSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                    }
                ]
            }
        """
        di_list_request = config_msgs.DIConfigList()
        ParseDict(di_config_list, di_list_request)
        response = self.config.SetDIConfigList(di_list_request)

        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_di_config_list(self):
        """
        DI Configuration List
            {
                'di_configs': [
                    {
                        'function_code': 2,
                        'function_name': "name",
                        'triggerSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}],
                        'successSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}],
                        'failureSignals': [{'address': 1, 'state': 1}, {'address': 2, 'state': 0}]
                    }
                ]
            }
        """
        response = self.config.GetDIConfigList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # FT Sensor Config
    ############################
    def set_ft_sensor_config(self,
                          dev_type, com_type, ip_address,
                               ft_frame_translation_offset_x=0.0,
                               ft_frame_translation_offset_y=0.0,
                               ft_frame_translation_offset_z=0.0,
                               ft_frame_rotation_offset_r=0.0,
                               ft_frame_rotation_offset_p=0.0,
                               ft_frame_rotation_offset_y=0.0,
                               arm_index=0):
        response = self.config.SetFTSensorConfig(config_msgs.FTSensorDevice(
            dev_type=dev_type, com_type=com_type,ip_address=ip_address,
            ft_frame_translation_offset_x=ft_frame_translation_offset_x,
            ft_frame_translation_offset_y=ft_frame_translation_offset_y,
            ft_frame_translation_offset_z=ft_frame_translation_offset_z,
            ft_frame_rotation_offset_r=ft_frame_rotation_offset_r,
            ft_frame_rotation_offset_p=ft_frame_rotation_offset_p,
            ft_frame_rotation_offset_y=ft_frame_rotation_offset_y,
            arm_index=arm_index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ft_sensor_config(self):
        response = self.config.GetFTSensorConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_ft_sensor_config_for(self, index: int):
        """
        F/T Sensor configuration at specific index
            index -> int
            returns FTSensorDevice
        """
        response = self.config.GetFTSensorConfigFor(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Auto Servo-Off
    ############################
    def set_auto_servo_off(self, enable: bool, time: float):
        """
        Auto Servo-Off Config
            enable -> bool
            time -> float
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
        Auto Servo-Off Config
            enable -> bool
            time -> float
        """
        response = self.config.GetAutoServoOff(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Safety Stop Config
    ############################
    def set_safety_stop_config(self, jpos_limit_stop_cat=StopCategory.CAT0,
                               jvel_limit_stop_cat=StopCategory.CAT0,
                               jtau_limit_stop_cat=StopCategory.CAT0,
                               tvel_limit_stop_cat=StopCategory.CAT0,
                               tforce_limit_stop_cat=StopCategory.CAT0,
                               power_limit_stop_cat=StopCategory.CAT0,
                               safegd_stop_cat=None,
                               safegd_type=None):
        """
        Safety Stop Category:
            jpos_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
            jvel_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
            jtau_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
            tvel_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
            tforce_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
            power_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        Optional:
            safegd_stop_cat -> list[StopCategory]
            safegd_type -> list[SafeGdType]
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
        Safety Stop Category:
            joint_position_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
            joint_speed_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
            joint_torque_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
            tcp_speed_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
            tcp_force_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
            power_limit_stop_cat = IMMEDIATE_BRAKE(0) | SMOOTH_BRAKE(1) | SMOOTH_ONLY(2)
        """
        response = self.config.GetSafetyStopConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Reduced Speed / Ratio
    ############################
    def get_reduced_ratio(self):
        response = self.config.GetReducedRatio(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_reduced_speed(self):
        response = self.config.GetReducedSpeed(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_reduced_speed(self, speed):
        response = self.config.SetReducedSpeed(config_msgs.SetReducedSpeedReq(speed=speed))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Teleop Params
    ############################
    def set_teleop_params(self, smooth_factor, cutoff_freq, error_gain):
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
        IO Data:
            smooth_factor   -> float
            cutoff_freq   -> float
            error_gain  -> float
        """
        response = self.config.GetTeleOpParams(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Kinematics
    ############################
    def get_kinematics_params(self):
        response = self.config.GetKinematicsParams(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Reference Frame Shortcuts
    ############################
    def load_reference_frame(self):
        response = self.config.GetRefFrameList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                    including_default_value_fields=True,
                                    preserving_proto_field_name=True,
                                    use_integers_for_enums=True)
        
    def save_reference_frame(self, frames, default_name):
        """
        frames = [
            {
                'name': 'frame1',
                'tpos': [0.0, 0.0, 0.0, 0.0, 0.0],
                'jpos0': [0.0, 0.0, 0.0, 0.0, 0.0]
            },
            {
                'name': 'frame2',
                'tpos': [0.0, 0.0, 0.0, 0.0, 0.0],
                'jpos1': [0.0, 0.0, 0.0, 0.0, 0.0]
            }
        ]
        default_name = "default_frame"
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

    ############################
    # Factory Reset / Control Gains
    ############################
    def restor_factory_control_gains(self):
        """Restore factory default control gains."""
        response = self.config.RestorFactoryControlGains(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # IMU Auto Mount
    ############################
    def get_imu_auto_mount(self):
        """
        IMU Auto Mounting Angles:
            ry -> float
            rz -> float
        """
        response = self.config.GetIMUAutoMount(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Tool Property List
    ############################
    def set_tool_property_list(self, entries: dict):
        """
        Set Tool Property List:
            {
                'tools': [
                    {
                        'name': 'tool_name',
                        'property': {
                            'mass': 1.0,
                            'center_of_mass': [0.0, 0.0, 0.0],
                            'inertia': [0.0, 0.0, 0.0],
                            'arm_index': 0
                        }
                    }
                ]
            }
        """
        req = config_msgs.ToolPropertyEntries()
        ParseDict(entries, req)
        response = self.config.SetToolPropertyList(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_property_list(self):
        """
        Get Tool Property List:
            returns ToolPropertyEntries
        """
        response = self.config.GetToolPropertyList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Joint Limit Config
    ############################
    def get_joint_limit_config(self):
        """
        Joint Limit Config:
            robot_name -> string
            joint_pos_limit -> { min: float[], max: float[] }
            arm_index -> int
            source_file -> string
        """
        response = self.config.GetJointLimitConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_joint_limit_config(self, config: dict):
        """
        Set Joint Limit Config:
            {
                'robot_name': 'name',
                'joint_pos_limit': { 'min': [...], 'max': [...] },
                'arm_index': 0
            }
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
        Get Original (factory) Joint Limit Config:
            robot_name -> string
            joint_pos_limit -> { min: float[], max: float[] }
            arm_index -> int
            source_file -> string
        """
        response = self.config.GetOriginalJointLimitConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Safety Snapshots
    ############################
    def save_safety_snapshot(self, label: str):
        """
        Save current safety configuration as a snapshot.
            label -> string
        Returns SafetySnapshotInfo:
            snapshot_id -> string
            label -> string
            created_at -> string
            immutable -> bool
        """
        response = self.config.SaveSafetySnapshot(
            config_msgs.SaveSafetySnapshotReq(label=label)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def list_safety_snapshots(self):
        """
        List all safety snapshots.
        Returns SafetySnapshotList:
            snapshots -> list of SafetySnapshotInfo
        """
        response = self.config.ListSafetySnapshots(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def restore_safety_snapshot(self, snapshot_id: str):
        """
        Restore a safety snapshot by ID.
            snapshot_id -> string
        """
        response = self.config.RestoreSafetySnapshot(
            config_msgs.SafetySnapshotId(snapshot_id=snapshot_id)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def delete_safety_snapshot(self, snapshot_id: str):
        """
        Delete a safety snapshot by ID.
            snapshot_id -> string
        """
        response = self.config.DeleteSafetySnapshot(
            config_msgs.SafetySnapshotId(snapshot_id=snapshot_id)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def restor_factory_safety_config(self):
        """Restore factory default safety configuration."""
        response = self.config.RestorFactorySafetyConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Operation Mode Config
    ############################
    def set_operation_mode_config(self, use_ext_auto_mode: bool = False,
                                  mode_locked: bool = False,
                                  auto_mode: bool = False):
        """
        Operation Mode Config:
            use_ext_auto_mode -> bool
            mode_locked -> bool
            auto_mode -> bool
        """
        response = self.config.SetOperationModeConfig(
            config_msgs.OperationModeConfig(
                use_ext_auto_mode=use_ext_auto_mode,
                mode_locked=mode_locked,
                auto_mode=auto_mode
            )
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_operation_mode_config(self):
        """
        Get Operation Mode Config:
            use_ext_auto_mode -> bool
            mode_locked -> bool
            auto_mode -> bool
        """
        response = self.config.GetOperationModeConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
