import sys
import os
import inspect
import importlib
from importlib.abc import MetaPathFinder, Loader
from importlib.machinery import ModuleSpec

# List of proto module
_PROTO_MODULES = {
    os.path.splitext(name)[0]
    for name in os.listdir(os.path.dirname(__file__))
    if name.endswith('.py') and name != '__init__.py'
}

def _is_caller_from_neuromeka():
    """if the import is from neuromeka package"""
    frame = inspect.currentframe()
    try:
        # Go up the call stack
        caller_frame = frame.f_back.f_back  # Skip this function and find_spec
        while caller_frame:
            name = caller_frame.f_globals.get('__name__', '')
            file = caller_frame.f_globals.get('__file__', '')
            if name.startswith('neuromeka.') or 'neuromeka' in file:
                return True
            caller_frame = caller_frame.f_back
        return False
    finally:
        del frame

class LocalProtoImporter(MetaPathFinder, Loader):
    """Redirects proto imports to local package"""
    
    def __init__(self, package_name):
        self.package_name = package_name
    
    def find_spec(self, fullname, path, target=None):
        """Check if it should handle this import"""
        if fullname in _PROTO_MODULES and _is_caller_from_neuromeka():
            return ModuleSpec(fullname, self, origin=self.package_name)
        return None
    
    def create_module(self, spec):
        return None
    
    def exec_module(self, module):
        """Load the module from local package"""
        local_module = importlib.import_module(f'.{module.__name__}', self.package_name)
        module.__dict__.update(local_module.__dict__)

# Install the import hook (only once)
_importer = LocalProtoImporter(__name__)
if _importer not in sys.meta_path:
    sys.meta_path.insert(0, _importer)

# Protocol service types
from .boot_pb2_grpc         import *
from .config_pb2_grpc       import *
from .control_pb2_grpc      import *
from .cri_pb2_grpc          import *
from .device_pb2_grpc       import *
from .ethercat_pb2_grpc     import *
from .linear_pb2_grpc       import *
from .mimic_pb2_grpc        import *
from .moby_pb2_grpc         import *
from .plotting_pb2_grpc     import *
from .rtde_pb2_grpc         import *
from .teleop_pb2_grpc       import *
from .teleop_dev_pb2_grpc   import *
from .welding_pb2_grpc      import *

# Protocol message types
from . import boot_msgs_pb2     as boot_msgs
from . import common_msgs_pb2   as common_msgs
from . import config_msgs_pb2   as config_msgs
from . import control_msgs_pb2  as control_msgs
from . import cri_msgs_pb2      as cri_msgs
from . import device_msgs_pb2   as device_msgs
from . import ethercat_msgs_pb2 as ethercat_msgs
from . import mimic_msgs_pb2    as mimic_msgs
from . import moby_msgs_pb2     as moby_msgs
from . import plotting_pb2      as plotting_msgs
from . import rtde_msgs_pb2     as rtde_msgs
from . import teleop_msgs_pb2   as teleop_msgs
from . import teleop_pb2        as teleop_services
from . import teleop_dev_pb2    as teleop_data
from . import welding_pb2       as welding_msgs
