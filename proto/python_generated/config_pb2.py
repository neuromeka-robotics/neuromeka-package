# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import config_msgs_pb2 as config__msgs__pb2
import common_msgs_pb2 as common__msgs__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='config.proto',
  package='Nrmk.IndyFramework',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0c\x63onfig.proto\x12\x12Nrmk.IndyFramework\x1a\x11\x63onfig_msgs.proto\x1a\x11\x63ommon_msgs.proto2\x9a#\n\x06\x43onfig\x12\x45\n\x0bGetRefFrame\x12\x19.Nrmk.IndyFramework.Empty\x1a\x19.Nrmk.IndyFramework.Frame\"\x00\x12H\n\x0bSetRefFrame\x12\x19.Nrmk.IndyFramework.Frame\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12W\n\x11SetRefFramePlanar\x12\x1f.Nrmk.IndyFramework.PlanarFrame\x1a\x1f.Nrmk.IndyFramework.FrameResult\"\x00\x12I\n\x0cSetToolFrame\x12\x19.Nrmk.IndyFramework.Frame\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12J\n\rSetSpeedRatio\x12\x19.Nrmk.IndyFramework.Ratio\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12S\n\x0fSetDIConfigList\x12 .Nrmk.IndyFramework.DIConfigList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12P\n\x0fGetDIConfigList\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.DIConfigList\"\x00\x12S\n\x0fSetDOConfigList\x12 .Nrmk.IndyFramework.DOConfigList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12P\n\x0fGetDOConfigList\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.DOConfigList\"\x00\x12O\n\x0fSetHomePosition\x12\x1c.Nrmk.IndyFramework.JointPos\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12L\n\x0fGetHomePosition\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1c.Nrmk.IndyFramework.JointPos\"\x00\x12L\n\x0fGetPackPosition\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1c.Nrmk.IndyFramework.JointPos\"\x00\x12Y\n\x0fSetAutoServoOff\x12&.Nrmk.IndyFramework.AutoServoOffConfig\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12V\n\x0fGetAutoServoOff\x12\x19.Nrmk.IndyFramework.Empty\x1a&.Nrmk.IndyFramework.AutoServoOffConfig\"\x00\x12W\n\x13SetJointControlGain\x12 .Nrmk.IndyFramework.JointGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12T\n\x13GetJointControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.JointGainSet\"\x00\x12U\n\x12SetTaskControlGain\x12\x1f.Nrmk.IndyFramework.TaskGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12R\n\x12GetTaskControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1f.Nrmk.IndyFramework.TaskGainSet\"\x00\x12_\n\x17SetImpedanceControlGain\x12$.Nrmk.IndyFramework.ImpedanceGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12\\\n\x17GetImpedanceControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a$.Nrmk.IndyFramework.ImpedanceGainSet\"\x00\x12W\n\x13SetForceControlGain\x12 .Nrmk.IndyFramework.ForceGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12T\n\x13GetForceControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.ForceGainSet\"\x00\x12U\n\x12SetTestControlGain\x12\x1f.Nrmk.IndyFramework.TestGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12R\n\x12GetTestControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1f.Nrmk.IndyFramework.TestGainSet\"\x00\x12Y\n\x14SetCustomControlGain\x12!.Nrmk.IndyFramework.CustomGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12V\n\x14GetCustomControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a!.Nrmk.IndyFramework.CustomGainSet\"\x00\x12g\n\x19SetNewControllerTestOnOff\x12*.Nrmk.IndyFramework.NewControllerTestState\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12i\n\x1eGetNewControllerTestOnOffState\x12\x19.Nrmk.IndyFramework.Empty\x1a*.Nrmk.IndyFramework.NewControllerTestState\"\x00\x12V\n\x0fSetFrictionComp\x12#.Nrmk.IndyFramework.FrictionCompSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12S\n\x0fGetFrictionComp\x12\x19.Nrmk.IndyFramework.Empty\x1a#.Nrmk.IndyFramework.FrictionCompSet\"\x00\x12Q\n\x0bSetMountPos\x12\".Nrmk.IndyFramework.MountingAngles\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12N\n\x0bGetMountPos\x12\x19.Nrmk.IndyFramework.Empty\x1a\".Nrmk.IndyFramework.MountingAngles\"\x00\x12U\n\x0fSetToolProperty\x12\".Nrmk.IndyFramework.ToolProperties\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12R\n\x0fGetToolProperty\x12\x19.Nrmk.IndyFramework.Empty\x1a\".Nrmk.IndyFramework.ToolProperties\"\x00\x12Z\n\x10SetCollSensLevel\x12&.Nrmk.IndyFramework.CollisionSensLevel\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12W\n\x10GetCollSensLevel\x12\x19.Nrmk.IndyFramework.Empty\x1a&.Nrmk.IndyFramework.CollisionSensLevel\"\x00\x12[\n\x10SetCollSensParam\x12\'.Nrmk.IndyFramework.CollisionThresholds\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12X\n\x10GetCollSensParam\x12\x19.Nrmk.IndyFramework.Empty\x1a\'.Nrmk.IndyFramework.CollisionThresholds\"\x00\x12T\n\rSetCollPolicy\x12#.Nrmk.IndyFramework.CollisionPolicy\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12Q\n\rGetCollPolicy\x12\x19.Nrmk.IndyFramework.Empty\x1a#.Nrmk.IndyFramework.CollisionPolicy\"\x00\x12S\n\x0fSetSafetyLimits\x12 .Nrmk.IndyFramework.SafetyLimits\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12P\n\x0fGetSafetyLimits\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.SafetyLimits\"\x00\x12[\n\x13SetSafetyStopConfig\x12$.Nrmk.IndyFramework.SafetyStopConfig\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12X\n\x13GetSafetyStopConfig\x12\x19.Nrmk.IndyFramework.Empty\x1a$.Nrmk.IndyFramework.SafetyStopConfig\"\x00\x12V\n\x0fGetReducedRatio\x12\x19.Nrmk.IndyFramework.Empty\x1a&.Nrmk.IndyFramework.GetReducedRatioRes\"\x00\x12V\n\x0fGetReducedSpeed\x12\x19.Nrmk.IndyFramework.Empty\x1a&.Nrmk.IndyFramework.GetReducedSpeedRes\"\x00\x12Y\n\x0fSetReducedSpeed\x12&.Nrmk.IndyFramework.SetReducedSpeedReq\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12W\n\x11SetFTSensorConfig\x12\".Nrmk.IndyFramework.FTSensorDevice\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12T\n\x11GetFTSensorConfig\x12\x19.Nrmk.IndyFramework.Empty\x1a\".Nrmk.IndyFramework.FTSensorDevice\"\x00\x12S\n\x0fSetTeleOpParams\x12 .Nrmk.IndyFramework.TeleOpParams\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12P\n\x0fGetTeleOpParams\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.TeleOpParams\"\x00\x12X\n\x13GetKinematicsParams\x12\x19.Nrmk.IndyFramework.Empty\x1a$.Nrmk.IndyFramework.KinematicsParams\"\x00\x62\x06proto3'
  ,
  dependencies=[config__msgs__pb2.DESCRIPTOR,common__msgs__pb2.DESCRIPTOR,])



_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_CONFIG = _descriptor.ServiceDescriptor(
  name='Config',
  full_name='Nrmk.IndyFramework.Config',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=75,
  serialized_end=4581,
  methods=[
  _descriptor.MethodDescriptor(
    name='GetRefFrame',
    full_name='Nrmk.IndyFramework.Config.GetRefFrame',
    index=0,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._FRAME,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetRefFrame',
    full_name='Nrmk.IndyFramework.Config.SetRefFrame',
    index=1,
    containing_service=None,
    input_type=config__msgs__pb2._FRAME,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetRefFramePlanar',
    full_name='Nrmk.IndyFramework.Config.SetRefFramePlanar',
    index=2,
    containing_service=None,
    input_type=config__msgs__pb2._PLANARFRAME,
    output_type=config__msgs__pb2._FRAMERESULT,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetToolFrame',
    full_name='Nrmk.IndyFramework.Config.SetToolFrame',
    index=3,
    containing_service=None,
    input_type=config__msgs__pb2._FRAME,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetSpeedRatio',
    full_name='Nrmk.IndyFramework.Config.SetSpeedRatio',
    index=4,
    containing_service=None,
    input_type=config__msgs__pb2._RATIO,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetDIConfigList',
    full_name='Nrmk.IndyFramework.Config.SetDIConfigList',
    index=5,
    containing_service=None,
    input_type=config__msgs__pb2._DICONFIGLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetDIConfigList',
    full_name='Nrmk.IndyFramework.Config.GetDIConfigList',
    index=6,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._DICONFIGLIST,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetDOConfigList',
    full_name='Nrmk.IndyFramework.Config.SetDOConfigList',
    index=7,
    containing_service=None,
    input_type=config__msgs__pb2._DOCONFIGLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetDOConfigList',
    full_name='Nrmk.IndyFramework.Config.GetDOConfigList',
    index=8,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._DOCONFIGLIST,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetHomePosition',
    full_name='Nrmk.IndyFramework.Config.SetHomePosition',
    index=9,
    containing_service=None,
    input_type=config__msgs__pb2._JOINTPOS,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetHomePosition',
    full_name='Nrmk.IndyFramework.Config.GetHomePosition',
    index=10,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._JOINTPOS,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetPackPosition',
    full_name='Nrmk.IndyFramework.Config.GetPackPosition',
    index=11,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._JOINTPOS,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetAutoServoOff',
    full_name='Nrmk.IndyFramework.Config.SetAutoServoOff',
    index=12,
    containing_service=None,
    input_type=config__msgs__pb2._AUTOSERVOOFFCONFIG,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetAutoServoOff',
    full_name='Nrmk.IndyFramework.Config.GetAutoServoOff',
    index=13,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._AUTOSERVOOFFCONFIG,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetJointControlGain',
    full_name='Nrmk.IndyFramework.Config.SetJointControlGain',
    index=14,
    containing_service=None,
    input_type=config__msgs__pb2._JOINTGAINSET,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetJointControlGain',
    full_name='Nrmk.IndyFramework.Config.GetJointControlGain',
    index=15,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._JOINTGAINSET,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetTaskControlGain',
    full_name='Nrmk.IndyFramework.Config.SetTaskControlGain',
    index=16,
    containing_service=None,
    input_type=config__msgs__pb2._TASKGAINSET,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetTaskControlGain',
    full_name='Nrmk.IndyFramework.Config.GetTaskControlGain',
    index=17,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._TASKGAINSET,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetImpedanceControlGain',
    full_name='Nrmk.IndyFramework.Config.SetImpedanceControlGain',
    index=18,
    containing_service=None,
    input_type=config__msgs__pb2._IMPEDANCEGAINSET,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetImpedanceControlGain',
    full_name='Nrmk.IndyFramework.Config.GetImpedanceControlGain',
    index=19,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._IMPEDANCEGAINSET,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetForceControlGain',
    full_name='Nrmk.IndyFramework.Config.SetForceControlGain',
    index=20,
    containing_service=None,
    input_type=config__msgs__pb2._FORCEGAINSET,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetForceControlGain',
    full_name='Nrmk.IndyFramework.Config.GetForceControlGain',
    index=21,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._FORCEGAINSET,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetTestControlGain',
    full_name='Nrmk.IndyFramework.Config.SetTestControlGain',
    index=22,
    containing_service=None,
    input_type=config__msgs__pb2._TESTGAINSET,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetTestControlGain',
    full_name='Nrmk.IndyFramework.Config.GetTestControlGain',
    index=23,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._TESTGAINSET,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetCustomControlGain',
    full_name='Nrmk.IndyFramework.Config.SetCustomControlGain',
    index=24,
    containing_service=None,
    input_type=config__msgs__pb2._CUSTOMGAINSET,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetCustomControlGain',
    full_name='Nrmk.IndyFramework.Config.GetCustomControlGain',
    index=25,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._CUSTOMGAINSET,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetNewControllerTestOnOff',
    full_name='Nrmk.IndyFramework.Config.SetNewControllerTestOnOff',
    index=26,
    containing_service=None,
    input_type=config__msgs__pb2._NEWCONTROLLERTESTSTATE,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetNewControllerTestOnOffState',
    full_name='Nrmk.IndyFramework.Config.GetNewControllerTestOnOffState',
    index=27,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._NEWCONTROLLERTESTSTATE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetFrictionComp',
    full_name='Nrmk.IndyFramework.Config.SetFrictionComp',
    index=28,
    containing_service=None,
    input_type=config__msgs__pb2._FRICTIONCOMPSET,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetFrictionComp',
    full_name='Nrmk.IndyFramework.Config.GetFrictionComp',
    index=29,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._FRICTIONCOMPSET,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetMountPos',
    full_name='Nrmk.IndyFramework.Config.SetMountPos',
    index=30,
    containing_service=None,
    input_type=config__msgs__pb2._MOUNTINGANGLES,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetMountPos',
    full_name='Nrmk.IndyFramework.Config.GetMountPos',
    index=31,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._MOUNTINGANGLES,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetToolProperty',
    full_name='Nrmk.IndyFramework.Config.SetToolProperty',
    index=32,
    containing_service=None,
    input_type=config__msgs__pb2._TOOLPROPERTIES,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetToolProperty',
    full_name='Nrmk.IndyFramework.Config.GetToolProperty',
    index=33,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._TOOLPROPERTIES,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetCollSensLevel',
    full_name='Nrmk.IndyFramework.Config.SetCollSensLevel',
    index=34,
    containing_service=None,
    input_type=config__msgs__pb2._COLLISIONSENSLEVEL,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetCollSensLevel',
    full_name='Nrmk.IndyFramework.Config.GetCollSensLevel',
    index=35,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._COLLISIONSENSLEVEL,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetCollSensParam',
    full_name='Nrmk.IndyFramework.Config.SetCollSensParam',
    index=36,
    containing_service=None,
    input_type=config__msgs__pb2._COLLISIONTHRESHOLDS,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetCollSensParam',
    full_name='Nrmk.IndyFramework.Config.GetCollSensParam',
    index=37,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._COLLISIONTHRESHOLDS,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetCollPolicy',
    full_name='Nrmk.IndyFramework.Config.SetCollPolicy',
    index=38,
    containing_service=None,
    input_type=config__msgs__pb2._COLLISIONPOLICY,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetCollPolicy',
    full_name='Nrmk.IndyFramework.Config.GetCollPolicy',
    index=39,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._COLLISIONPOLICY,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetSafetyLimits',
    full_name='Nrmk.IndyFramework.Config.SetSafetyLimits',
    index=40,
    containing_service=None,
    input_type=config__msgs__pb2._SAFETYLIMITS,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetSafetyLimits',
    full_name='Nrmk.IndyFramework.Config.GetSafetyLimits',
    index=41,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._SAFETYLIMITS,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetSafetyStopConfig',
    full_name='Nrmk.IndyFramework.Config.SetSafetyStopConfig',
    index=42,
    containing_service=None,
    input_type=config__msgs__pb2._SAFETYSTOPCONFIG,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetSafetyStopConfig',
    full_name='Nrmk.IndyFramework.Config.GetSafetyStopConfig',
    index=43,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._SAFETYSTOPCONFIG,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetReducedRatio',
    full_name='Nrmk.IndyFramework.Config.GetReducedRatio',
    index=44,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._GETREDUCEDRATIORES,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetReducedSpeed',
    full_name='Nrmk.IndyFramework.Config.GetReducedSpeed',
    index=45,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._GETREDUCEDSPEEDRES,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetReducedSpeed',
    full_name='Nrmk.IndyFramework.Config.SetReducedSpeed',
    index=46,
    containing_service=None,
    input_type=config__msgs__pb2._SETREDUCEDSPEEDREQ,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetFTSensorConfig',
    full_name='Nrmk.IndyFramework.Config.SetFTSensorConfig',
    index=47,
    containing_service=None,
    input_type=config__msgs__pb2._FTSENSORDEVICE,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetFTSensorConfig',
    full_name='Nrmk.IndyFramework.Config.GetFTSensorConfig',
    index=48,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._FTSENSORDEVICE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetTeleOpParams',
    full_name='Nrmk.IndyFramework.Config.SetTeleOpParams',
    index=49,
    containing_service=None,
    input_type=config__msgs__pb2._TELEOPPARAMS,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetTeleOpParams',
    full_name='Nrmk.IndyFramework.Config.GetTeleOpParams',
    index=50,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._TELEOPPARAMS,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetKinematicsParams',
    full_name='Nrmk.IndyFramework.Config.GetKinematicsParams',
    index=51,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=config__msgs__pb2._KINEMATICSPARAMS,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_CONFIG)

DESCRIPTOR.services_by_name['Config'] = _CONFIG

# @@protoc_insertion_point(module_scope)
