# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: rtde_msgs.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import device_msgs_pb2 as device__msgs__pb2
import common_msgs_pb2 as common__msgs__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0frtde_msgs.proto\x12\x12Nrmk.IndyFramework\x1a\x11\x64\x65vice_msgs.proto\x1a\x11\x63ommon_msgs.proto\"-\n\x0bTestRequest\x12\x0e\n\x06intVal\x18\x01 \x01(\x05\x12\x0e\n\x06strVal\x18\x02 \x01(\t\")\n\x0cTestResponse\x12\x0c\n\x04\x63ode\x18\x01 \x01(\x03\x12\x0b\n\x03msg\x18\x02 \x01(\t\"\xeb\x02\n\nMotionData\x12\x31\n\ntraj_state\x18\x01 \x01(\x0e\x32\x1d.Nrmk.IndyFramework.TrajState\x12\x15\n\rtraj_progress\x18\x02 \x01(\x05\x12\x14\n\x0cis_in_motion\x18\x03 \x01(\x08\x12\x19\n\x11is_target_reached\x18\x04 \x01(\x08\x12\x12\n\nis_pausing\x18\x05 \x01(\x08\x12\x13\n\x0bis_stopping\x18\x06 \x01(\x08\x12\x12\n\nhas_motion\x18\x07 \x01(\x08\x12\x13\n\x0bspeed_ratio\x18\x08 \x01(\x05\x12\x11\n\tmotion_id\x18\t \x01(\x05\x12\x17\n\x0fremain_distance\x18\n \x01(\x02\x12\x19\n\x11motion_queue_size\x18\x0b \x01(\r\x12\x19\n\x11\x63ur_traj_progress\x18\x0c \x01(\x05\x12.\n\x08response\x18\x64 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.Response\"\x9a\x02\n\x0b\x43ontrolData\x12\x15\n\rrunning_hours\x18\x01 \x01(\r\x12\x14\n\x0crunning_mins\x18\x02 \x01(\r\x12\x14\n\x0crunning_secs\x18\x03 \x01(\r\x12-\n\x08op_state\x18\x04 \x01(\x0e\x32\x1b.Nrmk.IndyFramework.OpState\x12\x10\n\x08sim_mode\x18\x05 \x01(\x08\x12\t\n\x01q\x18\n \x03(\x02\x12\x0c\n\x04qdot\x18\x0b \x03(\x02\x12\t\n\x01p\x18\x0c \x03(\x02\x12\x0c\n\x04pdot\x18\r \x03(\x02\x12\x11\n\tref_frame\x18\x14 \x03(\x02\x12\x12\n\ntool_frame\x18\x15 \x03(\x02\x12.\n\x08response\x18\x64 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.Response\"\x9f\x02\n\x0c\x43ontrolData2\x12\t\n\x01q\x18\x01 \x03(\x02\x12\x0c\n\x04qdot\x18\x02 \x03(\x02\x12\r\n\x05qddot\x18\x03 \x03(\x02\x12\x0c\n\x04qdes\x18\x04 \x03(\x02\x12\x0f\n\x07qdotdes\x18\x05 \x03(\x02\x12\x10\n\x08qddotdes\x18\x06 \x03(\x02\x12\t\n\x01p\x18\n \x03(\x02\x12\x0c\n\x04pdot\x18\x0b \x03(\x02\x12\r\n\x05pddot\x18\x0c \x03(\x02\x12\x0c\n\x04pdes\x18\r \x03(\x02\x12\x0f\n\x07pdotdes\x18\x0e \x03(\x02\x12\x10\n\x08pddotdes\x18\x0f \x03(\x02\x12\x0b\n\x03tau\x18\x14 \x03(\x02\x12\x0f\n\x07tau_act\x18\x15 \x03(\x02\x12\x0f\n\x07tau_ext\x18\x16 \x03(\x02\x12.\n\x08response\x18\x64 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.Response\"\xbc\x03\n\x06IOData\x12-\n\x02\x64i\x18\x01 \x03(\x0b\x32!.Nrmk.IndyFramework.DigitalSignal\x12-\n\x02\x64o\x18\x02 \x03(\x0b\x32!.Nrmk.IndyFramework.DigitalSignal\x12,\n\x02\x61i\x18\x03 \x03(\x0b\x32 .Nrmk.IndyFramework.AnalogSignal\x12,\n\x02\x61o\x18\x04 \x03(\x0b\x32 .Nrmk.IndyFramework.AnalogSignal\x12\x31\n\x06\x65nd_di\x18\x05 \x03(\x0b\x32!.Nrmk.IndyFramework.EndtoolSignal\x12\x31\n\x06\x65nd_do\x18\x06 \x03(\x0b\x32!.Nrmk.IndyFramework.EndtoolSignal\x12\x30\n\x06\x65nd_ai\x18\x07 \x03(\x0b\x32 .Nrmk.IndyFramework.AnalogSignal\x12\x30\n\x06\x65nd_ao\x18\x08 \x03(\x0b\x32 .Nrmk.IndyFramework.AnalogSignal\x12.\n\x08response\x18\x64 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.Response\"\x9f\x01\n\rViolationData\x12\x16\n\x0eviolation_code\x18\x01 \x01(\x04\x12\x0f\n\x07j_index\x18\x02 \x01(\r\x12\x0e\n\x06i_args\x18\x03 \x03(\x05\x12\x0e\n\x06\x66_args\x18\x04 \x03(\x02\x12\x15\n\rviolation_str\x18\x05 \x01(\t\x12.\n\x08response\x18\x64 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.Response\"\x83\x01\n\x15ViolationMessageQueue\x12:\n\x0fviolation_queue\x18\x01 \x03(\x0b\x32!.Nrmk.IndyFramework.ViolationData\x12.\n\x08response\x18\x64 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.Response\"\xb9\x01\n\tServoData\x12\x14\n\x0cstatus_codes\x18\x01 \x03(\t\x12\x14\n\x0ctemperatures\x18\x02 \x03(\x02\x12\x10\n\x08voltages\x18\x03 \x03(\x02\x12\x10\n\x08\x63urrents\x18\x04 \x03(\x02\x12\x15\n\rservo_actives\x18\n \x03(\x08\x12\x15\n\rbrake_actives\x18\x0b \x03(\x08\x12.\n\x08response\x18\x64 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.Response\"\xbb\x02\n\x0bProgramData\x12\x37\n\rprogram_state\x18\x01 \x01(\x0e\x32 .Nrmk.IndyFramework.ProgramState\x12\x0e\n\x06\x63md_id\x18\x02 \x01(\x05\x12\x12\n\nsub_cmd_id\x18\x03 \x01(\x05\x12\x15\n\rrunning_hours\x18\x04 \x01(\r\x12\x14\n\x0crunning_mins\x18\x05 \x01(\r\x12\x14\n\x0crunning_secs\x18\x06 \x01(\r\x12\x14\n\x0cprogram_name\x18\x07 \x01(\t\x12\x15\n\rprogram_alarm\x18\x08 \x01(\t\x12\x1a\n\x12program_annotation\x18\t \x01(\t\x12\x13\n\x0bspeed_ratio\x18\n \x01(\x05\x12.\n\x08response\x18\x64 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.Response\"\xd5\x01\n\tStopState\x12<\n\x08\x63\x61tegory\x18\x01 \x01(\x0e\x32*.Nrmk.IndyFramework.StopState.StopCategory\x12.\n\x08response\x18\x64 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.Response\"Z\n\x0cStopCategory\x12\x0e\n\nSTOP_CAT_0\x10\x00\x12\x0e\n\nSTOP_CAT_1\x10\x01\x12\x0e\n\nSTOP_CAT_2\x10\x02\x12\x1a\n\rSTOP_CAT_NONE\x10\xff\xff\xff\xff\xff\xff\xff\xff\xff\x01\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'rtde_msgs_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_TESTREQUEST']._serialized_start=77
  _globals['_TESTREQUEST']._serialized_end=122
  _globals['_TESTRESPONSE']._serialized_start=124
  _globals['_TESTRESPONSE']._serialized_end=165
  _globals['_MOTIONDATA']._serialized_start=168
  _globals['_MOTIONDATA']._serialized_end=531
  _globals['_CONTROLDATA']._serialized_start=534
  _globals['_CONTROLDATA']._serialized_end=816
  _globals['_CONTROLDATA2']._serialized_start=819
  _globals['_CONTROLDATA2']._serialized_end=1106
  _globals['_IODATA']._serialized_start=1109
  _globals['_IODATA']._serialized_end=1553
  _globals['_VIOLATIONDATA']._serialized_start=1556
  _globals['_VIOLATIONDATA']._serialized_end=1715
  _globals['_VIOLATIONMESSAGEQUEUE']._serialized_start=1718
  _globals['_VIOLATIONMESSAGEQUEUE']._serialized_end=1849
  _globals['_SERVODATA']._serialized_start=1852
  _globals['_SERVODATA']._serialized_end=2037
  _globals['_PROGRAMDATA']._serialized_start=2040
  _globals['_PROGRAMDATA']._serialized_end=2355
  _globals['_STOPSTATE']._serialized_start=2358
  _globals['_STOPSTATE']._serialized_end=2571
  _globals['_STOPSTATE_STOPCATEGORY']._serialized_start=2481
  _globals['_STOPSTATE_STOPCATEGORY']._serialized_end=2571
# @@protoc_insertion_point(module_scope)
