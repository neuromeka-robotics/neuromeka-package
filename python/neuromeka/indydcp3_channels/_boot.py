import sys
if sys.version_info >= (3, 9):
    from neuromeka.proto import *
else:
    from neuromeka.proto_step import *

from google.protobuf import json_format
from ._helpers import _message_to_dict


class BootChannelAPI:
    """ChannelAPI for Boot channel (port 20010/30010) methods."""

    def get_boot_status(self):
        response = self.boot.GetBootStatus(common_msgs.Empty())
        return _message_to_dict(response)
