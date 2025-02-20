# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: rtde.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import rtde_msgs_pb2 as rtde__msgs__pb2
import common_msgs_pb2 as common__msgs__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='rtde.proto',
  package='Nrmk.IndyFramework',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\nrtde.proto\x12\x12Nrmk.IndyFramework\x1a\x0frtde_msgs.proto\x1a\x11\x63ommon_msgs.proto2\xbb\x06\n\x0eRTDataExchange\x12L\n\rGetMotionData\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1e.Nrmk.IndyFramework.MotionData\"\x00\x12N\n\x0eGetControlData\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1f.Nrmk.IndyFramework.ControlData\"\x00\x12P\n\x0fGetControlState\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.ControlData2\"\x00\x12\x44\n\tGetIOData\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1a.Nrmk.IndyFramework.IOData\"\x00\x12J\n\x0cGetServoData\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1d.Nrmk.IndyFramework.ServoData\"\x00\x12R\n\x10GetViolationData\x12\x19.Nrmk.IndyFramework.Empty\x1a!.Nrmk.IndyFramework.ViolationData\"\x00\x12\x62\n\x18GetViolationMessageQueue\x12\x19.Nrmk.IndyFramework.Empty\x1a).Nrmk.IndyFramework.ViolationMessageQueue\"\x00\x12N\n\x0eGetProgramData\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1f.Nrmk.IndyFramework.ProgramData\"\x00\x12J\n\x0cGetStopState\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1d.Nrmk.IndyFramework.StopState\"\x00\x12S\n\x0cTestFunction\x12\x1f.Nrmk.IndyFramework.TestRequest\x1a .Nrmk.IndyFramework.TestResponse\"\x00\x62\x06proto3'
  ,
  dependencies=[rtde__msgs__pb2.DESCRIPTOR,common__msgs__pb2.DESCRIPTOR,])



_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_RTDATAEXCHANGE = _descriptor.ServiceDescriptor(
  name='RTDataExchange',
  full_name='Nrmk.IndyFramework.RTDataExchange',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=71,
  serialized_end=898,
  methods=[
  _descriptor.MethodDescriptor(
    name='GetMotionData',
    full_name='Nrmk.IndyFramework.RTDataExchange.GetMotionData',
    index=0,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=rtde__msgs__pb2._MOTIONDATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetControlData',
    full_name='Nrmk.IndyFramework.RTDataExchange.GetControlData',
    index=1,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=rtde__msgs__pb2._CONTROLDATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetControlState',
    full_name='Nrmk.IndyFramework.RTDataExchange.GetControlState',
    index=2,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=rtde__msgs__pb2._CONTROLDATA2,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetIOData',
    full_name='Nrmk.IndyFramework.RTDataExchange.GetIOData',
    index=3,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=rtde__msgs__pb2._IODATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetServoData',
    full_name='Nrmk.IndyFramework.RTDataExchange.GetServoData',
    index=4,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=rtde__msgs__pb2._SERVODATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetViolationData',
    full_name='Nrmk.IndyFramework.RTDataExchange.GetViolationData',
    index=5,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=rtde__msgs__pb2._VIOLATIONDATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetViolationMessageQueue',
    full_name='Nrmk.IndyFramework.RTDataExchange.GetViolationMessageQueue',
    index=6,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=rtde__msgs__pb2._VIOLATIONMESSAGEQUEUE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetProgramData',
    full_name='Nrmk.IndyFramework.RTDataExchange.GetProgramData',
    index=7,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=rtde__msgs__pb2._PROGRAMDATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetStopState',
    full_name='Nrmk.IndyFramework.RTDataExchange.GetStopState',
    index=8,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=rtde__msgs__pb2._STOPSTATE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='TestFunction',
    full_name='Nrmk.IndyFramework.RTDataExchange.TestFunction',
    index=9,
    containing_service=None,
    input_type=rtde__msgs__pb2._TESTREQUEST,
    output_type=rtde__msgs__pb2._TESTRESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_RTDATAEXCHANGE)

DESCRIPTOR.services_by_name['RTDataExchange'] = _RTDATAEXCHANGE

# @@protoc_insertion_point(module_scope)
