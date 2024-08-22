# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: config.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import config_msgs_pb2 as config__msgs__pb2
import common_msgs_pb2 as common__msgs__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0c\x63onfig.proto\x12\x12Nrmk.IndyFramework\x1a\x11\x63onfig_msgs.proto\x1a\x11\x63ommon_msgs.proto2\x9a#\n\x06\x43onfig\x12\x45\n\x0bGetRefFrame\x12\x19.Nrmk.IndyFramework.Empty\x1a\x19.Nrmk.IndyFramework.Frame\"\x00\x12H\n\x0bSetRefFrame\x12\x19.Nrmk.IndyFramework.Frame\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12W\n\x11SetRefFramePlanar\x12\x1f.Nrmk.IndyFramework.PlanarFrame\x1a\x1f.Nrmk.IndyFramework.FrameResult\"\x00\x12I\n\x0cSetToolFrame\x12\x19.Nrmk.IndyFramework.Frame\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12J\n\rSetSpeedRatio\x12\x19.Nrmk.IndyFramework.Ratio\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12S\n\x0fSetDIConfigList\x12 .Nrmk.IndyFramework.DIConfigList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12P\n\x0fGetDIConfigList\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.DIConfigList\"\x00\x12S\n\x0fSetDOConfigList\x12 .Nrmk.IndyFramework.DOConfigList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12P\n\x0fGetDOConfigList\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.DOConfigList\"\x00\x12O\n\x0fSetHomePosition\x12\x1c.Nrmk.IndyFramework.JointPos\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12L\n\x0fGetHomePosition\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1c.Nrmk.IndyFramework.JointPos\"\x00\x12L\n\x0fGetPackPosition\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1c.Nrmk.IndyFramework.JointPos\"\x00\x12Y\n\x0fSetAutoServoOff\x12&.Nrmk.IndyFramework.AutoServoOffConfig\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12V\n\x0fGetAutoServoOff\x12\x19.Nrmk.IndyFramework.Empty\x1a&.Nrmk.IndyFramework.AutoServoOffConfig\"\x00\x12W\n\x13SetJointControlGain\x12 .Nrmk.IndyFramework.JointGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12T\n\x13GetJointControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.JointGainSet\"\x00\x12U\n\x12SetTaskControlGain\x12\x1f.Nrmk.IndyFramework.TaskGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12R\n\x12GetTaskControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1f.Nrmk.IndyFramework.TaskGainSet\"\x00\x12_\n\x17SetImpedanceControlGain\x12$.Nrmk.IndyFramework.ImpedanceGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12\\\n\x17GetImpedanceControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a$.Nrmk.IndyFramework.ImpedanceGainSet\"\x00\x12W\n\x13SetForceControlGain\x12 .Nrmk.IndyFramework.ForceGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12T\n\x13GetForceControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.ForceGainSet\"\x00\x12U\n\x12SetTestControlGain\x12\x1f.Nrmk.IndyFramework.TestGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12R\n\x12GetTestControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1f.Nrmk.IndyFramework.TestGainSet\"\x00\x12Y\n\x14SetCustomControlGain\x12!.Nrmk.IndyFramework.CustomGainSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12V\n\x14GetCustomControlGain\x12\x19.Nrmk.IndyFramework.Empty\x1a!.Nrmk.IndyFramework.CustomGainSet\"\x00\x12g\n\x19SetNewControllerTestOnOff\x12*.Nrmk.IndyFramework.NewControllerTestState\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12i\n\x1eGetNewControllerTestOnOffState\x12\x19.Nrmk.IndyFramework.Empty\x1a*.Nrmk.IndyFramework.NewControllerTestState\"\x00\x12V\n\x0fSetFrictionComp\x12#.Nrmk.IndyFramework.FrictionCompSet\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12S\n\x0fGetFrictionComp\x12\x19.Nrmk.IndyFramework.Empty\x1a#.Nrmk.IndyFramework.FrictionCompSet\"\x00\x12Q\n\x0bSetMountPos\x12\".Nrmk.IndyFramework.MountingAngles\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12N\n\x0bGetMountPos\x12\x19.Nrmk.IndyFramework.Empty\x1a\".Nrmk.IndyFramework.MountingAngles\"\x00\x12U\n\x0fSetToolProperty\x12\".Nrmk.IndyFramework.ToolProperties\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12R\n\x0fGetToolProperty\x12\x19.Nrmk.IndyFramework.Empty\x1a\".Nrmk.IndyFramework.ToolProperties\"\x00\x12Z\n\x10SetCollSensLevel\x12&.Nrmk.IndyFramework.CollisionSensLevel\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12W\n\x10GetCollSensLevel\x12\x19.Nrmk.IndyFramework.Empty\x1a&.Nrmk.IndyFramework.CollisionSensLevel\"\x00\x12[\n\x10SetCollSensParam\x12\'.Nrmk.IndyFramework.CollisionThresholds\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12X\n\x10GetCollSensParam\x12\x19.Nrmk.IndyFramework.Empty\x1a\'.Nrmk.IndyFramework.CollisionThresholds\"\x00\x12T\n\rSetCollPolicy\x12#.Nrmk.IndyFramework.CollisionPolicy\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12Q\n\rGetCollPolicy\x12\x19.Nrmk.IndyFramework.Empty\x1a#.Nrmk.IndyFramework.CollisionPolicy\"\x00\x12S\n\x0fSetSafetyLimits\x12 .Nrmk.IndyFramework.SafetyLimits\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12P\n\x0fGetSafetyLimits\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.SafetyLimits\"\x00\x12[\n\x13SetSafetyStopConfig\x12$.Nrmk.IndyFramework.SafetyStopConfig\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12X\n\x13GetSafetyStopConfig\x12\x19.Nrmk.IndyFramework.Empty\x1a$.Nrmk.IndyFramework.SafetyStopConfig\"\x00\x12V\n\x0fGetReducedRatio\x12\x19.Nrmk.IndyFramework.Empty\x1a&.Nrmk.IndyFramework.GetReducedRatioRes\"\x00\x12V\n\x0fGetReducedSpeed\x12\x19.Nrmk.IndyFramework.Empty\x1a&.Nrmk.IndyFramework.GetReducedSpeedRes\"\x00\x12Y\n\x0fSetReducedSpeed\x12&.Nrmk.IndyFramework.SetReducedSpeedReq\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12W\n\x11SetFTSensorConfig\x12\".Nrmk.IndyFramework.FTSensorDevice\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12T\n\x11GetFTSensorConfig\x12\x19.Nrmk.IndyFramework.Empty\x1a\".Nrmk.IndyFramework.FTSensorDevice\"\x00\x12S\n\x0fSetTeleOpParams\x12 .Nrmk.IndyFramework.TeleOpParams\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12P\n\x0fGetTeleOpParams\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.TeleOpParams\"\x00\x12X\n\x13GetKinematicsParams\x12\x19.Nrmk.IndyFramework.Empty\x1a$.Nrmk.IndyFramework.KinematicsParams\"\x00\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'config_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_CONFIG']._serialized_start=75
  _globals['_CONFIG']._serialized_end=4581
# @@protoc_insertion_point(module_scope)
