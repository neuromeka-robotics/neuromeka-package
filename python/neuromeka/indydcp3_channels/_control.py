import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *

from neuromeka.common import *
from neuromeka.enums import *

from typing import Optional, List

from google.protobuf import json_format
from google.protobuf.json_format import ParseDict


class ControlMixin:
    """Mixin for Control channel (port 20001/30001) methods."""

    ############################
    # Motion Control (Move commands)
    ############################
    def stop_motion(self, stop_category=StopCategory.CAT2) -> dict:
        """
         stop motion element:
            stop_category -> StopCategory
                CAT0  = 0
                CAT1  = 1
                CAT2  = 2
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
         Joint Move:
            blending_type -> BlendingType.Type
                NONE
                OVERRIDE
                DUPLICATE
            base_type -> JointBaseType
                ABSOLUTE
                RELATIVE
            vel_ratio (0-100) -> int
            acc_ratio (0-100) -> int
            post_condition -> PostCondition
            teaching_mode -> bool

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
        jtarget = [deg, deg, deg, deg, deg, deg]
        move_time = seconds
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
              bypass_singular=False,
              arm_index: int = 0) -> dict:
        """
        tstart = [mm, mm, mm, deg, deg, deg]
        ttarget = [mm, mm, mm, deg, deg, deg]

            base_tye -> TaskBaseType
                ABSOLUTE
                RELATIVE
                TCP
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
                ),
            )

        response = self.control.MoveL(control_msgs.MoveLReq(
            target=ptarget,
            blending=blending,
            vel_ratio=vel_ratio, acc_ratio=acc_ratio,
            post_condition=post_cond,
            teaching_mode=teaching_mode,
            bypass_singular=bypass_singular,
            arm_index=arm_index
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
                   post_condition=PostCondition(),
                   arm_index: int = 0) -> dict:
        """
        tstart = [mm, mm, mm, deg, deg, deg]
        ttarget = [mm, mm, mm, deg, deg, deg]

            base_tye -> TaskBaseType
                ABSOLUTE
                RELATIVE
                TCP
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
                ),
            )

        response = self.control.MoveLT(control_msgs.MoveLTReq(
            target=ptarget,
            blending=blending,
            time=move_time,
            post_condition=post_cond,
            arm_index=arm_index
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
        tstart = [mm, mm, mm, deg, deg, deg]
        ttarget = [mm, mm, mm, deg, deg, deg]
         Recover from violation
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
        ft_Fx -> float N
        ft_Fy -> float N
        ft_Fz -> float N
        ft_Tx -> float N*m
        ft_Ty -> float N*m
        ft_Tz -> float N*m
        """
        response = self.control.GetTransformedFTSensorData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_transformed_ft_sensor_data_for(self, index: int):
        response = self.control.GetTransformedFTSensorDataFor(common_msgs.Int(value=index))
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
              bypass_singular=False,
              arm_index: int = 0) -> dict:
        """
        tstart = [mm, mm, mm, deg, deg, deg]
        ttarget = [mm, mm, mm, deg, deg, deg]
         Recover from violation
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
            bypass_singular=bypass_singular,
            arm_index=arm_index
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
               post_condition=PostCondition(),
               arm_index: int = 0) -> dict:
        
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
                ),
            )

        response = self.control.MoveCT(control_msgs.MoveCTReq(
            target=ctarget,
            blending=blending,
            angle=angle,
            setting_type=setting_type,
            move_type=move_type,
            time=move_time,
            post_condition=post_cond,
            arm_index=arm_index
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def move_gcode(self, gcode_file,
                   is_smooth_mode=False,
                   smooth_radius=0.0,
                   vel_ratio=Limits.JogVelRatioDefault,
                   acc_ratio=Limits.JogAccRatioDefault,
                   arm_index: int = 0) -> dict:
        
        gcode_req = control_msgs.MoveGcodeReq(gcode_file=gcode_file,
                                              is_smooth_mode=is_smooth_mode,
                                              smooth_radius=smooth_radius,
                                              vel_ratio=vel_ratio,
                                              acc_ratio=acc_ratio,
                                              arm_index=arm_index)
        
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
        post_cond = control_msgs.MotionCondition()
        if post_condition is not None:
            post_cond = control_msgs.MotionCondition(
                type_cond=post_condition.condition_type,
                type_react=post_condition.reaction_type,
                const_cond=post_condition.const_cond,
                io_cond=control_msgs.IOCondition(
                    di=self.__to_digital_request_list__(
                        [{'address': di[0], 'state': di[1]} for di in post_condition.digital_inputs]),
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
    # Motion Control (Teleoperation)
    ############################
    def get_teleop_device(self):
        response = self.control.GetTeleOpDevice(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_teleop_state(self):
        response = self.control.GetTeleOpState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def connect_teleop_device(self, name: str, type: control_msgs.TeleOpDevice, ip: str, port: int):
        response = self.control.ConnectTeleOpDevice(
            control_msgs.TeleOpDevice(name=name,type=type,ip=ip,port=port)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def disconnect_teleop_device(self):
        response = self.control.DisConnectTeleOpDevice(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def read_teleop_input(self):
        response = self.control.ReadTeleOpInput(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def read_teleop_input_for(self, index: int):
        """Read teleoperation input for specific index."""
        response = self.control.ReadTeleOpInputFor(common_msgs.Int(value=index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    
    def start_teleop(self, method):
        """
        Start tele op
        method:
            TELE_TASK_ABSOLUTE = 0
            TELE_TASK_RELATIVE = 1
            TELE_JOINT_ABSOLUTE = 10
            TELE_JOINT_RELATIVE = 11
        """
        response = self.control.StartTeleOp(
            control_msgs.TeleOpState(mode=control_msgs.TeleMode.TELE_RAW, method=method))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def stop_teleop(self):
        """
        Stop tele op
        """
        response = self.control.StopTeleOp(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_play_rate(self, rate: float):
        response = self.control.SetPlayRate(control_msgs.TelePlayRate(rate=rate))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_play_rate(self):
        response = self.control.GetPlayRate(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tele_file_list(self):
        response = self.control.GetTeleFileList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def save_tele_motion(self, name: str):
        response = self.control.SaveTeleMotion(control_msgs.TeleFileReq(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def load_tele_motion(self, name: str):
        response = self.control.LoadTeleMotion(control_msgs.TeleFileReq(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def delete_tele_motion(self, name: str):
        response = self.control.DeleteTeleMotion(control_msgs.TeleFileReq(name=name))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def enable_tele_key(self, enable):
        response = self.control.EnableTeleKey(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movetelej_abs(self, jpos, vel_ratio=1.0, acc_ratio=1.0):
        """
        Joint Teleoperation - Absolute
        jpos = [deg, deg, deg, deg, deg, deg]
        """
        response = self.control.MoveTeleJ(control_msgs.MoveTeleJReq(jpos=jpos, vel_ratio=vel_ratio, acc_ratio=acc_ratio,
                                                                    method=control_msgs.TELE_JOINT_ABSOLUTE))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movetelej_rel(self, jpos, vel_ratio=1.0, acc_ratio=1.0):
        """
        Joint Teleoperation - Relative
        jpos = [deg, deg, deg, deg, deg, deg]
        """
        response = self.control.MoveTeleJ(control_msgs.MoveTeleJReq(jpos=jpos, vel_ratio=vel_ratio, acc_ratio=acc_ratio,
                                                                    method=control_msgs.TELE_JOINT_RELATIVE))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movetelel_abs(self, tpos, vel_ratio=1.0, acc_ratio=1.0, arm_index=0):
        """
        Task Teleoperation - Absolute
        jpos = [mm, mm, mm, deg, deg, deg]
        """
        response = self.control.MoveTeleL(control_msgs.MoveTeleLReq(tpos=tpos, vel_ratio=vel_ratio, acc_ratio=acc_ratio,
                                                                    method=control_msgs.TELE_TASK_ABSOLUTE, arm_index=arm_index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def movetelel_rel(self, tpos, vel_ratio=1.0, acc_ratio=1.0, arm_index=0):
        """
        Task Teleoperation - Relative
        jpos = [mm, mm, mm, deg, deg, deg]
        """
        response = self.control.MoveTeleL(control_msgs.MoveTeleLReq(tpos=tpos, vel_ratio=vel_ratio, acc_ratio=acc_ratio,
                                                                    method=control_msgs.TELE_TASK_RELATIVE, arm_index=arm_index))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
    
    def move_axis(self, start_mm, target_mm, is_absolute=True, vel_ratio=5, acc_ratio=100, teaching_mode=False):
        """
        start_mm = [mm, mm, mm] -> pos
        target_mm = [mm, mm, mm] -> pos
        vel_mm : int -> vel_ratio
        acc_mm : int -> acc_ratio
        is_absolute : True if target is absolute -> base_type
        """
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
    # Kinematics & Teaching
    ############################
    def inverse_kin(self, tpos, init_jpos, arm_index=0) -> dict:
        """
        :param tpos:
        :param init_jpos:
        :return:
            'jpos': []
        """
        response = self.control.InverseKinematics(control_msgs.InverseKinematicsReq(
            tpos=list(tpos),
            init_jpos=list(init_jpos),
            arm_index=arm_index
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_motion_j(self, req: dict):
        msg = control_msgs.GetMotionJReq()
        try:
            ParseDict(req, msg)
        except Exception as e:
            return {'error': f'parse_error: {e}', 'input': req}
        response = self.control.GetMotionJ(msg)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_motion_l(self, req: dict):
        msg = control_msgs.GetMotionLReq()
        try:
            ParseDict(req, msg)
        except Exception as e:
            return {'error': f'parse_error: {e}', 'input': req}
        response = self.control.GetMotionL(msg)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_motion_c(self, req: dict):
        msg = control_msgs.GetMotionCReq()
        try:
            ParseDict(req, msg)
        except Exception as e:
            return {'error': f'parse_error: {e}', 'input': req}
        response = self.control.GetMotionC(msg)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
        
    def forward_kin(self, jpos, arm_index=0) -> dict:
        """
        :param tpos:
        :param init_jpos:
        :return:
            'jpos': []
        """
        response = self.control.ForwardKinematics(control_msgs.ForwardKinematicsReq(
            jpos=list(jpos), arm_index=arm_index
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_direct_teaching(self, enable=True) -> dict:
        """
         enable = True | False
        """
        response = self.control.SetDirectTeaching(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_simulation_mode(self, enable=True) -> dict:
        """
         Set simulation mode = True | False
        """
        response = self.control.SetSimulationMode(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def recover(self) -> dict:
        """
         Recover from violation
        """
        response = self.control.Recover(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_manual_recovery(self, enable=True) -> dict:
        """
         Set manual recovery = True | False
        """
        response = self.control.SetManualRecovery(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def calculate_relative_pose(self, start_pos, end_pos,
                                base_type=TaskBaseType.ABSOLUTE):
        """
        Calculate relative pose
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
        Calculate current pos rel
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
         Play program
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
         Pause program
        """
        response = self.control.PauseProgram(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def resume_program(self):
        """
         Resume program
        """
        response = self.control.ResumeProgram(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def stop_program(self):
        """
         Stop program
        """
        response = self.control.StopProgram(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    # --- Program Breakpoints & Stepping ---
    def set_program_breakpoints(self, breakpoints: dict):
        msg = common_msgs.ProgramBreakPoints()
        try:
            ParseDict(breakpoints, msg)
        except Exception as e:
            return {'error': f'parse_error: {e}', 'input': breakpoints}
        response = self.control.SetProgramBreakPoints(msg)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_program_breakpoints(self):
        response = self.control.GetProgramBreakPoints(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def program_step_over(self):
        response = self.control.ProgramStepOver(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def program_step_into(self):
        response = self.control.ProgramStepInto(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def program_step_out(self):
        response = self.control.ProgramStepOut(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_tact_time(self, type: str, tact_time: float):
        """
        TactTime
            type -> str {not implemented yet}
            tact_time -> float {seconds}
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
        TactTime
            type -> str {not implemented yet}
            tact_time -> float {seconds}
        """
        response = self.control.GetTactTime(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Compliance Mode
    ############################
    def set_compliance_mode(self, enable: bool, stiffness: 'Optional[List[int]]' = None):
        """
        Set Compliance Mode
            enable -> bool
            stiffness -> int32[] (per-axis stiffness levels)
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
        Get Compliance Mode
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
    def get_bool_variable(self):
        """
        Bool Variables:
            [
                addr -> int32
                value -> bool
            ]
        """
        response = self.control.GetBoolVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_int_variable(self):
        """
        Integer Variables:
            [
                addr -> int32
                value -> int32
            ]
        """
        response = self.control.GetIntVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_float_variable(self):
        """
        Float Variables:
            [
                addr -> int32
                value -> float
            ]
        """
        response = self.control.GetFloatVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_jpos_variable(self):
        """
        JPos Variables:
            [
                addr -> int32
                jpos -> float[]
            ]
        """
        response = self.control.GetJPosVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)['variables']

    def get_tpos_variable(self):
        """
        TPos Variables:
            [
                addr -> int32
                tpos -> float[]
            ]
        """
        response = self.control.GetTPosVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_bool_variable(self, bool_variables: list):
        """
        Bool Variables:
            [
                addr -> int32
                value -> bool
            ]
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
        Integer Variables:
            [
                addr -> int32
                value -> int64
            ]
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
        Float Variables:
            [
                addr -> int32
                value -> float
            ]
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
        JPos Variables:
            [
                addr -> int32
                jpos -> float[]
            ]
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
        TPos Variables:
            [
                addr -> int32
                tpos -> float[]
            ]
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
        response = self.control.SetPluginBoolVariable(
            common_msgs.NamedBool(name=name, value=value)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plugin_bool_variable(self, name: str):
        response = self.control.GetPluginBoolVariable(
            common_msgs.Name(name=name)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_plugin_int_variable(self, name: str, value: int):
        response = self.control.SetPluginIntVariable(
            common_msgs.NamedInt(name=name, value=value)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plugin_int_variable(self, name: str):
        response = self.control.GetPluginIntVariable(
            common_msgs.Name(name=name)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_plugin_float_variable(self, name: str, value: float):
        response = self.control.SetPluginFloatVariable(
            common_msgs.NamedFloat(name=name, value=value)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plugin_float_variable(self, name: str):
        response = self.control.GetPluginFloatVariable(
            common_msgs.Name(name=name)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_plugin_jpos_variable(self, name: str, jpos: List[float]):
        response = self.control.SetPluginJPosVariable(
            common_msgs.NamedJointPosition(name=name, jpos=jpos)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plugin_jpos_variable(self, name: str):
        response = self.control.GetPluginJPosVariable(
            common_msgs.Name(name=name)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_plugin_tpos_variable(self, name: str, tpos: List[float]):
        response = self.control.SetPluginTPosVariable(
            common_msgs.NamedTaskPosition(name=name, tpos=tpos)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plugin_tpos_variable(self, name: str):
        response = self.control.GetPluginTPosVariable(
            common_msgs.Name(name=name)
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
        Set Force Mode using a dict matching control_msgs.ForceModeReq schema.
        Tip: Use get_force_mode() to see the shape and modify as needed.
        """
        msg = control_msgs.ForceModeReq()
        ParseDict(force_mode, msg)
        response = self.control.SetForceMode(msg)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_force_mode(self):
        response = self.control.GetForceMode(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Friction Compensation State
    ############################
    def set_friction_comp_state(self, enable=False) -> dict:
        response = self.control.SetFrictionCompensation(common_msgs.State(enable=enable))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_friction_comp_state(self) -> dict:
        response = self.control.GetFrictionCompensationState(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # IndySDK related
    ############################
    def activate_sdk(self, license_key, expire_date):
        """
        license_key: license key issued by Neuromeka
        expire_date: expire date for the license, format YYYY-MM-DD
        SDKLicenseResp:
            activated -> bool, True if activated
            response (code, msg)
                - 0, 'Activated'                -> SDK Activated
                - 1, 'Invalid'                  -> Wrong key or expire date
                - 2, 'No Internet Connection'   -> Need Internet for License Verification
                - 3, 'Expired'                  -> License Expired
                - 4, 'HW_FAILURE'               -> Failed acquire HW ID to verify license
        """
        response = self.control.ActivateIndySDK(
            control_msgs.SDKLicenseInfo(license_key=license_key, expire_date=expire_date))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_custom_control_mode(self, mode):
        """
        mode:
        - False (0): IndyFramework's default controller is used
        - True (1): IndySDK's component is used
        """
        response = self.control.SetCustomControlMode(common_msgs.IntMode(mode=mode))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_custom_control_mode(self):
        """

        """
        response = self.control.GetCustomControlMode(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Wait Commands
    ############################
    def wait_io(self, 
                di_signal_list, 
                do_signal_list, 
                end_di_signal_list, 
                end_do_signal_list, 
                conjunction=0):
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
         Wait time [s]
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
         Wait progress [s]
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
         Wait trajectory
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
         Wait radius [mm]
        """
        response = self.control.WaitRadius(control_msgs.WaitRadiusReq(
            radius=radius
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Misc Control
    ############################
    def move_recover_joint(self, jtarget,
                           base_type=JointBaseType.ABSOLUTE) -> dict:
        """
         Move recover joint
         jtarget = [deg, deg, deg, deg, deg, deg]
        """
        response = self.control.MoveRecoverJoint(
            control_msgs.TargetJ(j_target=list(jtarget), base_type=base_type)
        )
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_control_info(self):
        response = self.control.GetControlInfo(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def check_aproach_retract_valid(self, tpos, init_jpos, pre_tpos, post_tpos, arm_index=0):
        """
        Check aproach retract valid
        """
        response = self.control.CheckAproachRetractValid(control_msgs.CheckAproachRetractValidReq(
            tpos=list(tpos),
            init_jpos=list(init_jpos),
            pre_tpos=list(pre_tpos),
            post_tpos=list(post_tpos),
            arm_index=arm_index
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_pallet_point_list(self, tpos, jpos, pre_tpos, post_tpos, pallet_pattern, width, height, arm_index=0):
        """
        Get pallet point list
        """
        response = self.control.GetPalletPointList(control_msgs.GetPalletPointListReq(
            tpos=list(tpos),
            jpos=list(jpos),
            pre_tpos=list(pre_tpos),
            post_tpos=list(post_tpos),
            pallet_pattern=pallet_pattern,
            width=width,
            height=height,
            arm_index=arm_index
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def play_tuning_program(self, prog_name: str = '', prog_idx: int = -1,
                            tuning_space=common_msgs.TUNE_ALL, precision=common_msgs.HIGH_PRECISION,
                            vel_level_max=9):
        """
        Play tuning program
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

        ParseDict(tuning_prog_dict, tuning_req)
        response = self.control.PlayTuningProgram(tuning_req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def ping_from_conty(self):
        response = self.control.PingFromConty(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                    including_default_value_fields=True,
                                    preserving_proto_field_name=True,
                                    use_integers_for_enums=True)

    def get_ft_zero(self):
        response = self.control.FTZero(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
        
    def get_inference_data(self):
        response = self.control.GetControlInferenceData(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_inference_data(self, *args):        

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

    ############################
    # IO Variable
    ############################
    def set_io_variable(self, io_vars_to_set: list):
        """
        Set IO Variables:
            io_vars_to_set -> list of dict
                [{'addr': 0, 'value': 1.0, 'signal_type': 0}, ...]
            signal_type: DI=0, DO=1, AI=2, AO=3, EndDI=4, EndDO=5, EndAI=6, EndAO=7
        """
        io_request_list = []
        for io_var_item in io_vars_to_set:
            io_request_list.append(control_msgs.IOVariable(
                addr=io_var_item['addr'],
                value=io_var_item.get('value', 0.0),
                signal_type=io_var_item.get('signal_type', 0)
            ))
        self.control.SetIOVariable(control_msgs.IOVars(variables=io_request_list))

    def get_io_variable(self):
        """
        Get IO Variables:
            returns list of IOVariable { addr, value, signal_type }
        """
        response = self.control.GetIOVariable(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    ############################
    # Program Line
    ############################
    def play_program_line(self, prog_name: str = '', prog_idx: int = -1,
                          start_cmd_id: int = 0, start_sub_cmd_id: int = 0,
                          end_cmd_id: int = 0, end_sub_cmd_id: int = 0,
                          teaching_mode: bool = False, debug_mode: bool = False):
        """
        Play a specific program line.
            prog_name -> string
            prog_idx -> int
            start_cmd_id -> int (0 means start line is not set)
            start_sub_cmd_id -> int
            end_cmd_id -> int (0 means end line is not set)
            end_sub_cmd_id -> int
            teaching_mode -> bool
            debug_mode -> bool
        """
        response = self.control.PlayProgramLine(control_msgs.Program(
            prog_name=prog_name,
            prog_idx=prog_idx,
            start_line_index=common_msgs.ProgramLine(cmd_id=start_cmd_id, sub_cmd_id=start_sub_cmd_id),
            end_line_index=common_msgs.ProgramLine(cmd_id=end_cmd_id, sub_cmd_id=end_sub_cmd_id),
            teaching_mode=teaching_mode,
            debug_mode=debug_mode
        ))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
