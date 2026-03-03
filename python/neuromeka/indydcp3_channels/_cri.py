import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *

from google.protobuf import json_format


class CRIMixin:
    """Mixin for CRI channel (port 20181/30181) methods."""

    def activate_cri(self, on: bool) -> dict:
        response = self.cri.ActiveCRIVel(common_msgs.State(enable=on))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def is_cri_active(self) -> dict:
        response = self.cri.IsSFDLogin(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def login_cri_server(self, email: str, token: str) -> dict:
        response = self.cri.LoginSFD(cri_msgs.SFDAccount(email=email, token=token))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def is_cri_login(self) -> dict:
        response = self.cri.IsSFDLogin(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_cri_target(self, pn: str, fn: str, rn: str) -> dict:
        response = self.cri.SelectSFDTarget(cri_msgs.SFDTarget(pn=pn, fn=fn, rn=rn))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def set_cri_option(self, on: bool) -> dict:
        response = self.cri.ActiveCRIVel(common_msgs.State(enable=on))
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_cri_proj_list(self) -> dict:
        response = self.cri.GetSFDProjList(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)

    def get_cri(self) -> dict:
        response = self.cri.GetCRI(common_msgs.Empty())
        return json_format.MessageToDict(response,
                                         including_default_value_fields=True,
                                         preserving_proto_field_name=True,
                                         use_integers_for_enums=True)
