import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *

from google.protobuf import json_format
from ._helpers import _message_to_dict


class CRIChannelAPI:
    """ChannelAPI for CRI channel (port 20181/30181) methods."""

    def activate_cri(self, on: bool) -> dict:
        response = self.cri.ActiveCRIVel(common_msgs.State(enable=on))
        return _message_to_dict(response)

    def is_cri_active(self) -> dict:
        response = self.cri.IsSFDLogin(common_msgs.Empty())
        return _message_to_dict(response)

    def login_cri_server(self, email: str, token: str) -> dict:
        response = self.cri.LoginSFD(cri_msgs.SFDAccount(email=email, token=token))
        return _message_to_dict(response)

    def is_cri_login(self) -> dict:
        response = self.cri.IsSFDLogin(common_msgs.Empty())
        return _message_to_dict(response)

    def set_cri_target(self, pn: str, fn: str, rn: str) -> dict:
        response = self.cri.SelectSFDTarget(cri_msgs.SFDTarget(pn=pn, fn=fn, rn=rn))
        return _message_to_dict(response)

    def set_cri_option(self, on: bool) -> dict:
        response = self.cri.ActiveCRIVel(common_msgs.State(enable=on))
        return _message_to_dict(response)

    def get_cri_proj_list(self) -> dict:
        response = self.cri.GetSFDProjList(common_msgs.Empty())
        return _message_to_dict(response)

    def get_cri(self) -> dict:
        response = self.cri.GetCRI(common_msgs.Empty())
        return _message_to_dict(response)

    def logout_sfd(self):
        return _message_to_dict(self.cri.LogoutSFD(common_msgs.Empty()))

    def save_sfd_login_info(self, request):
        return _message_to_dict(self.cri.SaveSFDLoginInfo(request))

    def load_sfd_login_info(self):
        return _message_to_dict(self.cri.LoadSFDLoginInfo(common_msgs.Empty()))

    def get_sfd_login_info(self):
        return _message_to_dict(self.cri.GetSFDLoginInfo(common_msgs.Empty()))

    def generate_sfd_token(self, request):
        return _message_to_dict(self.cri.GenerateSFDToken(request))

    def save_sfd_auto_set(self, request):
        return _message_to_dict(self.cri.SaveSFDAutoSet(request))

    def load_sfd_auto_set(self):
        return _message_to_dict(self.cri.LoadSFDAutoSet(common_msgs.Empty()))

    def release_sfd_target(self):
        return _message_to_dict(self.cri.ReleaseSFDTarget(common_msgs.Empty()))

    def get_sfd_target(self):
        return _message_to_dict(self.cri.GetSFDTarget(common_msgs.Empty()))

    def is_sfd_target_valid(self):
        return _message_to_dict(self.cri.IsSFDTargetValid(common_msgs.Empty()))

    def start_cri_record(self):
        return _message_to_dict(self.cri.StartCRIRecord(common_msgs.Empty()))

    def stop_cri_record(self):
        return _message_to_dict(self.cri.StopCRIRecord(common_msgs.Empty()))

    def start_cri_playback(self):
        return _message_to_dict(self.cri.StartCRIPlayback(common_msgs.Empty()))

    def stop_cri_playback(self):
        return _message_to_dict(self.cri.StopCRIPlayback(common_msgs.Empty()))

    def get_cri_record_mode(self):
        return _message_to_dict(self.cri.GetCRIRecordMode(common_msgs.Empty()))
