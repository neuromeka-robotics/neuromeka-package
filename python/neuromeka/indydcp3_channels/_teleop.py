import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *

from google.protobuf import json_format


class TeleopChannelAPI:
    """ChannelAPI for TeleOp channel methods."""

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

    def get_self_collision_pairs(self):
        response = self.teleop.GetSelfCollisionPairs(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_plane_constraint_config(self):
        response = self.teleop.GetPlaneConstraintConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_static_obstacle_constraint_config(self):
        response = self.teleop.GetStaticObstacleConstraintConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_dynamic_obstacle_constraint_config(self):
        response = self.teleop.GetDynamicObstacleConstraintConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_orientation_deviation_config(self):
        response = self.teleop.GetOrientationDeviationConfig(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

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
