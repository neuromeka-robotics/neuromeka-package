import json
import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *

from google.protobuf import json_format


class TeleopChannelAPI:
    """ChannelAPI for TeleOp channel methods."""

    @staticmethod
    def _to_dict(response):
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    @staticmethod
    def _parse_dict(message_cls, payload):
        req = message_cls()
        json_format.ParseDict(payload, req)
        return req

    def set_obstacle_info(self, obstacle_info: dict):
        req = teleop_msgs.ObstacleInfo()
        json_format.ParseDict(obstacle_info, req)
        response = self.teleop.SetObstacleInfo(req)
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_obstacle_info(self, idx: int):
        response = self.teleop.GetObstacleInfo(teleop_msgs.ObstacleIndex(idx=idx))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_collision_spheres(self):
        response = self.teleop.GetCollisionSpheres(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_joint_constraint_config(self):
        response = self.teleop.GetJointConstraintConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_joint_constraint_config(self, joint_constraint_config: dict):
        req = self._parse_dict(teleop_msgs.JointConstraintConfig, joint_constraint_config)
        response = self.teleop.SetJointConstraintConfig(req)
        return self._to_dict(response)

    def get_self_collision_pairs(self):
        response = self.teleop.GetSelfCollisionPairs(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_self_collision_pairs(self, self_collision_pairs: dict):
        req = self._parse_dict(teleop_msgs.SelfCollisionPairs, self_collision_pairs)
        response = self.teleop.SetSelfCollisionPairs(req)
        return self._to_dict(response)

    def get_self_collision_config(self):
        response = self.teleop.GetSelfCollisionConfig(common_msgs.Empty())
        return self._to_dict(response)

    def set_self_collision_config(self, self_collision_config: dict):
        req = self._parse_dict(teleop_msgs.SelfCollisionConstraintConfig, self_collision_config)
        response = self.teleop.SetSelfCollisionConfig(req)
        return self._to_dict(response)

    def get_plane_constraint_config(self):
        response = self.teleop.GetPlaneConstraintConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_plane_constraint_config(self, plane_constraint_config: dict):
        req = self._parse_dict(teleop_msgs.PlaneConstraintConfig, plane_constraint_config)
        response = self.teleop.SetPlaneConstraintConfig(req)
        return self._to_dict(response)

    def get_static_obstacle_constraint_config(self):
        response = self.teleop.GetStaticObstacleConstraintConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_static_obstacle_constraint_config(self, static_obstacle_constraint_config: dict):
        req = self._parse_dict(teleop_msgs.ObstacleConstraintConfig, static_obstacle_constraint_config)
        response = self.teleop.SetStaticObstacleConstraintConfig(req)
        return self._to_dict(response)

    def get_dynamic_obstacle_constraint_config(self):
        response = self.teleop.GetDynamicObstacleConstraintConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_dynamic_obstacle_constraint_config(self, dynamic_obstacle_constraint_config: dict):
        req = self._parse_dict(teleop_msgs.ObstacleConstraintConfig, dynamic_obstacle_constraint_config)
        response = self.teleop.SetDynamicObstacleConstraintConfig(req)
        return self._to_dict(response)

    def get_orientation_deviation_config(self):
        response = self.teleop.GetOrientationDeviationConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_orientation_deviation_config(self, orientation_deviation_config: dict):
        req = self._parse_dict(teleop_msgs.OrientationDeviationConfig, orientation_deviation_config)
        response = self.teleop.SetOrientationDeviationConfig(req)
        return self._to_dict(response)

    def apply_teleop_constraint_batch(self, constraint_batch: dict):
        req = self._parse_dict(teleop_msgs.ApplyTeleopConstraintBatchRequest, constraint_batch)
        response = self.teleop.ApplyTeleopConstraintBatch(req)
        return self._to_dict(response)

    def get_teleop_constraint_runtime_status(self):
        response = self.teleop.GetTeleopConstraintRuntimeStatus(common_msgs.Empty())
        return self._to_dict(response)

    def get_teleop_tuning_params(self):
        response = self.teleop.GetTeleopTuningParams(common_msgs.Empty())
        return self._to_dict(response)

    def set_teleop_tuning_params(self, teleop_tuning_params: dict = None,
                                 spring_energy_limit=None,
                                 spring_energy_damping_onset_ratio=None,
                                 spring_energy_damping_onset=None):
        if teleop_tuning_params is not None:
            req = self._parse_dict(teleop_msgs.TeleopTuningParams, teleop_tuning_params)
        else:
            req = teleop_msgs.TeleopTuningParams()
            if spring_energy_limit is not None:
                req.spring_energy_limit = float(spring_energy_limit)
            if spring_energy_damping_onset_ratio is not None:
                req.spring_energy_damping_onset_ratio = float(spring_energy_damping_onset_ratio)
            if spring_energy_damping_onset is not None:
                req.spring_energy_damping_onset = float(spring_energy_damping_onset)
        response = self.teleop.SetTeleopTuningParams(req)
        return self._to_dict(response)

    def get_desired_position(self):
        response = self.teleop.GetDesiredPosition(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_current_position(self):
        response = self.teleop.GetCurrentPosition(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_tool_collision_sphere_config(self):
        response = self.teleop.GetToolCollisionSphereConfig(common_msgs.Empty())
        return self._to_dict(response)

    def set_tool_collision_sphere_config(self, tool_collision_sphere_config):
        req = common_msgs.Message()
        if isinstance(tool_collision_sphere_config, str):
            req.content = tool_collision_sphere_config
        elif isinstance(tool_collision_sphere_config, dict):
            if "content" in tool_collision_sphere_config:
                json_format.ParseDict(tool_collision_sphere_config, req)
            else:
                req.content = json.dumps(tool_collision_sphere_config)
        else:
            raise TypeError("tool_collision_sphere_config must be a dict or JSON string")
        response = self.teleop.SetToolCollisionSphereConfig(req)
        return self._to_dict(response)
