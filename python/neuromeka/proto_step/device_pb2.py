# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: device.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import device_msgs_pb2 as device__msgs__pb2
import common_msgs_pb2 as common__msgs__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='device.proto',
  package='Nrmk.IndyFramework',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0c\x64\x65vice.proto\x12\x12Nrmk.IndyFramework\x1a\x11\x64\x65vice_msgs.proto\x1a\x11\x63ommon_msgs.proto2\x89!\n\x06\x44\x65vice\x12J\n\tSetBrakes\x12\x1d.Nrmk.IndyFramework.MotorList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12H\n\x0bSetServoAll\x12\x19.Nrmk.IndyFramework.State\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12\x45\n\x08SetServo\x12\x19.Nrmk.IndyFramework.Servo\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12H\n\x05SetDI\x12\x1f.Nrmk.IndyFramework.DigitalList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12H\n\x05SetDO\x12\x1f.Nrmk.IndyFramework.DigitalList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12Q\n\x08SetEndDI\x12%.Nrmk.IndyFramework.EndtoolSignalList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12Q\n\x08SetEndDO\x12%.Nrmk.IndyFramework.EndtoolSignalList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12G\n\x05SetAI\x12\x1e.Nrmk.IndyFramework.AnalogList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12G\n\x05SetAO\x12\x1e.Nrmk.IndyFramework.AnalogList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12J\n\x08SetEndAI\x12\x1e.Nrmk.IndyFramework.AnalogList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12J\n\x08SetEndAO\x12\x1e.Nrmk.IndyFramework.AnalogList\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12\x44\n\x0b\x45xecuteTool\x12\x18.Nrmk.IndyFramework.Name\x1a\x19.Nrmk.IndyFramework.Empty\"\x00\x12S\n\rSetEndRS485Rx\x12\".Nrmk.IndyFramework.EndtoolRS485Rx\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12P\n\rGetEndRS485Rx\x12\x19.Nrmk.IndyFramework.Empty\x1a\".Nrmk.IndyFramework.EndtoolRS485Rx\"\x00\x12P\n\rGetEndRS485Tx\x12\x19.Nrmk.IndyFramework.Empty\x1a\".Nrmk.IndyFramework.EndtoolRS485Tx\"\x00\x12J\n\x0cSetEndLedDim\x12\x1d.Nrmk.IndyFramework.EndLedDim\x1a\x19.Nrmk.IndyFramework.Empty\"\x00\x12\x45\n\x05GetDI\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1f.Nrmk.IndyFramework.DigitalList\"\x00\x12\x45\n\x05GetDO\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1f.Nrmk.IndyFramework.DigitalList\"\x00\x12N\n\x08GetEndDI\x12\x19.Nrmk.IndyFramework.Empty\x1a%.Nrmk.IndyFramework.EndtoolSignalList\"\x00\x12N\n\x08GetEndDO\x12\x19.Nrmk.IndyFramework.Empty\x1a%.Nrmk.IndyFramework.EndtoolSignalList\"\x00\x12\x44\n\x05GetAI\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1e.Nrmk.IndyFramework.AnalogList\"\x00\x12\x44\n\x05GetAO\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1e.Nrmk.IndyFramework.AnalogList\"\x00\x12G\n\x08GetEndAI\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1e.Nrmk.IndyFramework.AnalogList\"\x00\x12G\n\x08GetEndAO\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1e.Nrmk.IndyFramework.AnalogList\"\x00\x12K\n\tGetEL5001\x12\x19.Nrmk.IndyFramework.Empty\x1a!.Nrmk.IndyFramework.GetEL5001Data\"\x00\x12K\n\tGetEL5101\x12\x19.Nrmk.IndyFramework.Empty\x1a!.Nrmk.IndyFramework.GetEL5101Data\"\x00\x12L\n\rGetDeviceInfo\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1e.Nrmk.IndyFramework.DeviceInfo\"\x00\x12Z\n\x14GetBrakeControlStyle\x12\x19.Nrmk.IndyFramework.Empty\x1a%.Nrmk.IndyFramework.BrakeControlStyle\"\x00\x12U\n\x10SetSanderCommand\x12!.Nrmk.IndyFramework.SanderCommand\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12R\n\x10GetSanderCommand\x12\x19.Nrmk.IndyFramework.Empty\x1a!.Nrmk.IndyFramework.SanderCommand\"\x00\x12P\n\x0fGetFTSensorData\x12\x19.Nrmk.IndyFramework.Empty\x1a .Nrmk.IndyFramework.FTSensorData\"\x00\x12H\n\x0bGetConveyor\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1c.Nrmk.IndyFramework.Conveyor\"\x00\x12K\n\x0fSetConveyorName\x12\x18.Nrmk.IndyFramework.Name\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12M\n\x11SetConveyorByName\x12\x18.Nrmk.IndyFramework.Name\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12Q\n\x12SetConveyorEncoder\x12\x1b.Nrmk.IndyFramework.Encoder\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12Q\n\x12SetConveyorTrigger\x12\x1b.Nrmk.IndyFramework.Trigger\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12N\n\x11SetConveyorOffset\x12\x19.Nrmk.IndyFramework.Float\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12W\n\x17SetConveyorStartingPose\x12\x1c.Nrmk.IndyFramework.PosePair\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12W\n\x17SetConveyorTerminalPose\x12\x1c.Nrmk.IndyFramework.PosePair\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12R\n\x10GetConveyorState\x12\x19.Nrmk.IndyFramework.Empty\x1a!.Nrmk.IndyFramework.ConveyorState\"\x00\x12T\n\x0eGetLoadFactors\x12\x19.Nrmk.IndyFramework.Empty\x1a%.Nrmk.IndyFramework.GetLoadFactorsRes\"\x00\x12W\n\x0bSetAutoMode\x12\".Nrmk.IndyFramework.SetAutoModeReq\x1a\".Nrmk.IndyFramework.SetAutoModeRes\"\x00\x12R\n\rCheckAutoMode\x12\x19.Nrmk.IndyFramework.Empty\x1a$.Nrmk.IndyFramework.CheckAutoModeRes\"\x00\x12X\n\x10\x43heckReducedMode\x12\x19.Nrmk.IndyFramework.Empty\x1a\'.Nrmk.IndyFramework.CheckReducedModeRes\"\x00\x12^\n\x16GetSafetyFunctionState\x12\x19.Nrmk.IndyFramework.Empty\x1a\'.Nrmk.IndyFramework.SafetyFunctionState\"\x00\x12`\n\x15RequestSafetyFunction\x12\'.Nrmk.IndyFramework.SafetyFunctionState\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12Z\n\x14GetSafetyControlData\x12\x19.Nrmk.IndyFramework.Empty\x1a%.Nrmk.IndyFramework.SafetyControlData\"\x00\x12N\n\x0eGetGripperData\x12\x19.Nrmk.IndyFramework.Empty\x1a\x1f.Nrmk.IndyFramework.GripperData\"\x00\x12T\n\x11SetGripperCommand\x12\".Nrmk.IndyFramework.GripperCommand\x1a\x19.Nrmk.IndyFramework.Empty\"\x00\x12\x65\n\x15\x41\x64\x64PhotoneoCalibPoint\x12,.Nrmk.IndyFramework.AddPhotoneoCalibPointReq\x1a\x1c.Nrmk.IndyFramework.Response\"\x00\x12]\n\x14GetPhotoneoDetection\x12!.Nrmk.IndyFramework.VisionRequest\x1a .Nrmk.IndyFramework.VisionResult\"\x00\x12]\n\x14GetPhotoneoRetrieval\x12!.Nrmk.IndyFramework.VisionRequest\x1a .Nrmk.IndyFramework.VisionResult\"\x00\x62\x06proto3'
  ,
  dependencies=[device__msgs__pb2.DESCRIPTOR,common__msgs__pb2.DESCRIPTOR,])



_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_DEVICE = _descriptor.ServiceDescriptor(
  name='Device',
  full_name='Nrmk.IndyFramework.Device',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=75,
  serialized_end=4308,
  methods=[
  _descriptor.MethodDescriptor(
    name='SetBrakes',
    full_name='Nrmk.IndyFramework.Device.SetBrakes',
    index=0,
    containing_service=None,
    input_type=device__msgs__pb2._MOTORLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetServoAll',
    full_name='Nrmk.IndyFramework.Device.SetServoAll',
    index=1,
    containing_service=None,
    input_type=common__msgs__pb2._STATE,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetServo',
    full_name='Nrmk.IndyFramework.Device.SetServo',
    index=2,
    containing_service=None,
    input_type=device__msgs__pb2._SERVO,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetDI',
    full_name='Nrmk.IndyFramework.Device.SetDI',
    index=3,
    containing_service=None,
    input_type=device__msgs__pb2._DIGITALLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetDO',
    full_name='Nrmk.IndyFramework.Device.SetDO',
    index=4,
    containing_service=None,
    input_type=device__msgs__pb2._DIGITALLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetEndDI',
    full_name='Nrmk.IndyFramework.Device.SetEndDI',
    index=5,
    containing_service=None,
    input_type=device__msgs__pb2._ENDTOOLSIGNALLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetEndDO',
    full_name='Nrmk.IndyFramework.Device.SetEndDO',
    index=6,
    containing_service=None,
    input_type=device__msgs__pb2._ENDTOOLSIGNALLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetAI',
    full_name='Nrmk.IndyFramework.Device.SetAI',
    index=7,
    containing_service=None,
    input_type=device__msgs__pb2._ANALOGLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetAO',
    full_name='Nrmk.IndyFramework.Device.SetAO',
    index=8,
    containing_service=None,
    input_type=device__msgs__pb2._ANALOGLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetEndAI',
    full_name='Nrmk.IndyFramework.Device.SetEndAI',
    index=9,
    containing_service=None,
    input_type=device__msgs__pb2._ANALOGLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetEndAO',
    full_name='Nrmk.IndyFramework.Device.SetEndAO',
    index=10,
    containing_service=None,
    input_type=device__msgs__pb2._ANALOGLIST,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='ExecuteTool',
    full_name='Nrmk.IndyFramework.Device.ExecuteTool',
    index=11,
    containing_service=None,
    input_type=common__msgs__pb2._NAME,
    output_type=common__msgs__pb2._EMPTY,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetEndRS485Rx',
    full_name='Nrmk.IndyFramework.Device.SetEndRS485Rx',
    index=12,
    containing_service=None,
    input_type=common__msgs__pb2._ENDTOOLRS485RX,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetEndRS485Rx',
    full_name='Nrmk.IndyFramework.Device.GetEndRS485Rx',
    index=13,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=common__msgs__pb2._ENDTOOLRS485RX,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetEndRS485Tx',
    full_name='Nrmk.IndyFramework.Device.GetEndRS485Tx',
    index=14,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=common__msgs__pb2._ENDTOOLRS485TX,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetEndLedDim',
    full_name='Nrmk.IndyFramework.Device.SetEndLedDim',
    index=15,
    containing_service=None,
    input_type=device__msgs__pb2._ENDLEDDIM,
    output_type=common__msgs__pb2._EMPTY,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetDI',
    full_name='Nrmk.IndyFramework.Device.GetDI',
    index=16,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._DIGITALLIST,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetDO',
    full_name='Nrmk.IndyFramework.Device.GetDO',
    index=17,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._DIGITALLIST,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetEndDI',
    full_name='Nrmk.IndyFramework.Device.GetEndDI',
    index=18,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._ENDTOOLSIGNALLIST,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetEndDO',
    full_name='Nrmk.IndyFramework.Device.GetEndDO',
    index=19,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._ENDTOOLSIGNALLIST,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetAI',
    full_name='Nrmk.IndyFramework.Device.GetAI',
    index=20,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._ANALOGLIST,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetAO',
    full_name='Nrmk.IndyFramework.Device.GetAO',
    index=21,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._ANALOGLIST,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetEndAI',
    full_name='Nrmk.IndyFramework.Device.GetEndAI',
    index=22,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._ANALOGLIST,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetEndAO',
    full_name='Nrmk.IndyFramework.Device.GetEndAO',
    index=23,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._ANALOGLIST,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetEL5001',
    full_name='Nrmk.IndyFramework.Device.GetEL5001',
    index=24,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._GETEL5001DATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetEL5101',
    full_name='Nrmk.IndyFramework.Device.GetEL5101',
    index=25,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._GETEL5101DATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetDeviceInfo',
    full_name='Nrmk.IndyFramework.Device.GetDeviceInfo',
    index=26,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._DEVICEINFO,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetBrakeControlStyle',
    full_name='Nrmk.IndyFramework.Device.GetBrakeControlStyle',
    index=27,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._BRAKECONTROLSTYLE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetSanderCommand',
    full_name='Nrmk.IndyFramework.Device.SetSanderCommand',
    index=28,
    containing_service=None,
    input_type=device__msgs__pb2._SANDERCOMMAND,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetSanderCommand',
    full_name='Nrmk.IndyFramework.Device.GetSanderCommand',
    index=29,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._SANDERCOMMAND,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetFTSensorData',
    full_name='Nrmk.IndyFramework.Device.GetFTSensorData',
    index=30,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._FTSENSORDATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetConveyor',
    full_name='Nrmk.IndyFramework.Device.GetConveyor',
    index=31,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._CONVEYOR,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetConveyorName',
    full_name='Nrmk.IndyFramework.Device.SetConveyorName',
    index=32,
    containing_service=None,
    input_type=common__msgs__pb2._NAME,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetConveyorByName',
    full_name='Nrmk.IndyFramework.Device.SetConveyorByName',
    index=33,
    containing_service=None,
    input_type=common__msgs__pb2._NAME,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetConveyorEncoder',
    full_name='Nrmk.IndyFramework.Device.SetConveyorEncoder',
    index=34,
    containing_service=None,
    input_type=device__msgs__pb2._ENCODER,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetConveyorTrigger',
    full_name='Nrmk.IndyFramework.Device.SetConveyorTrigger',
    index=35,
    containing_service=None,
    input_type=device__msgs__pb2._TRIGGER,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetConveyorOffset',
    full_name='Nrmk.IndyFramework.Device.SetConveyorOffset',
    index=36,
    containing_service=None,
    input_type=common__msgs__pb2._FLOAT,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetConveyorStartingPose',
    full_name='Nrmk.IndyFramework.Device.SetConveyorStartingPose',
    index=37,
    containing_service=None,
    input_type=common__msgs__pb2._POSEPAIR,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetConveyorTerminalPose',
    full_name='Nrmk.IndyFramework.Device.SetConveyorTerminalPose',
    index=38,
    containing_service=None,
    input_type=common__msgs__pb2._POSEPAIR,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetConveyorState',
    full_name='Nrmk.IndyFramework.Device.GetConveyorState',
    index=39,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._CONVEYORSTATE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetLoadFactors',
    full_name='Nrmk.IndyFramework.Device.GetLoadFactors',
    index=40,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._GETLOADFACTORSRES,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetAutoMode',
    full_name='Nrmk.IndyFramework.Device.SetAutoMode',
    index=41,
    containing_service=None,
    input_type=device__msgs__pb2._SETAUTOMODEREQ,
    output_type=device__msgs__pb2._SETAUTOMODERES,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='CheckAutoMode',
    full_name='Nrmk.IndyFramework.Device.CheckAutoMode',
    index=42,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._CHECKAUTOMODERES,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='CheckReducedMode',
    full_name='Nrmk.IndyFramework.Device.CheckReducedMode',
    index=43,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._CHECKREDUCEDMODERES,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetSafetyFunctionState',
    full_name='Nrmk.IndyFramework.Device.GetSafetyFunctionState',
    index=44,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._SAFETYFUNCTIONSTATE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='RequestSafetyFunction',
    full_name='Nrmk.IndyFramework.Device.RequestSafetyFunction',
    index=45,
    containing_service=None,
    input_type=device__msgs__pb2._SAFETYFUNCTIONSTATE,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetSafetyControlData',
    full_name='Nrmk.IndyFramework.Device.GetSafetyControlData',
    index=46,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._SAFETYCONTROLDATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetGripperData',
    full_name='Nrmk.IndyFramework.Device.GetGripperData',
    index=47,
    containing_service=None,
    input_type=common__msgs__pb2._EMPTY,
    output_type=device__msgs__pb2._GRIPPERDATA,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='SetGripperCommand',
    full_name='Nrmk.IndyFramework.Device.SetGripperCommand',
    index=48,
    containing_service=None,
    input_type=device__msgs__pb2._GRIPPERCOMMAND,
    output_type=common__msgs__pb2._EMPTY,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='AddPhotoneoCalibPoint',
    full_name='Nrmk.IndyFramework.Device.AddPhotoneoCalibPoint',
    index=49,
    containing_service=None,
    input_type=device__msgs__pb2._ADDPHOTONEOCALIBPOINTREQ,
    output_type=common__msgs__pb2._RESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetPhotoneoDetection',
    full_name='Nrmk.IndyFramework.Device.GetPhotoneoDetection',
    index=50,
    containing_service=None,
    input_type=device__msgs__pb2._VISIONREQUEST,
    output_type=device__msgs__pb2._VISIONRESULT,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='GetPhotoneoRetrieval',
    full_name='Nrmk.IndyFramework.Device.GetPhotoneoRetrieval',
    index=51,
    containing_service=None,
    input_type=device__msgs__pb2._VISIONREQUEST,
    output_type=device__msgs__pb2._VISIONRESULT,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_DEVICE)

DESCRIPTOR.services_by_name['Device'] = _DEVICE

# @@protoc_insertion_point(module_scope)
