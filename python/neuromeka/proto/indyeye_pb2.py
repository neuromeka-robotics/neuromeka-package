# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: indyeye.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\rindyeye.proto\x12\x1eIndyFramework.Protobuf.EyeTask\"(\n\x05Point\x12\t\n\x01x\x18\x01 \x01(\x02\x12\t\n\x01y\x18\x02 \x01(\x02\x12\t\n\x01z\x18\x03 \x01(\x02\"\x96\x01\n\x10WeldingLinesInfo\x12\x0c\n\x04name\x18\x01 \x01(\t\x12:\n\x0bstart_point\x18\x02 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\x12\x38\n\tend_point\x18\x03 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\"%\n\x17WeldingLinesInfoRequest\x12\n\n\x02id\x18\x01 \x01(\x05\"\x92\x01\n\x18WeldingLinesInfoResponse\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x41\n\x07welding\x18\x02 \x03(\x0b\x32\x30.IndyFramework.Protobuf.EyeTask.WeldingLinesInfo\x12\x12\n\ncell_types\x18\x03 \x03(\t\x12\x13\n\x0b\x65rror_state\x18\x04 \x01(\x08\"%\n\x17StraightLineInfoRequest\x12\n\n\x02id\x18\x01 \x01(\x05\"\xef\x01\n\x18StraightLineInfoResponse\x12\n\n\x02id\x18\x01 \x01(\x05\x12<\n\rnormal_vector\x18\x02 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\x12:\n\x0bstart_point\x18\x03 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\x12\x38\n\tend_point\x18\x04 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\x12\x13\n\x0b\x65rror_state\x18\x05 \x01(\x08\"\xc9\x01\n\x17\x43ircularLineThreePoints\x12:\n\x0bstart_point\x18\x01 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\x12\x38\n\tvia_point\x18\x02 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\x12\x38\n\tend_point\x18\x03 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\"g\n\x18\x43ircularLineCenterRadius\x12;\n\x0c\x63\x65nter_point\x18\x01 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\x12\x0e\n\x06radius\x18\x02 \x01(\x02\"%\n\x17\x43ircularLineInfoRequest\x12\n\n\x02id\x18\x01 \x01(\x05\"\xda\x02\n\x18\x43ircularLineInfoResponse\x12\n\n\x02id\x18\x01 \x01(\x05\x12<\n\rnormal_vector\x18\x02 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\x12?\n\x10reference_vector\x18\x03 \x01(\x0b\x32%.IndyFramework.Protobuf.EyeTask.Point\x12M\n\x0cpoint_method\x18\x04 \x01(\x0b\x32\x37.IndyFramework.Protobuf.EyeTask.CircularLineThreePoints\x12O\n\rvector_method\x18\x05 \x01(\x0b\x32\x38.IndyFramework.Protobuf.EyeTask.CircularLineCenterRadius\x12\x13\n\x0b\x65rror_state\x18\x06 \x01(\x08\"\x1f\n\x11ServerInfoRequest\x12\n\n\x02id\x18\x01 \x01(\x05\"]\n\x12ServerInfoResponse\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x0f\n\x07version\x18\x02 \x01(\t\x12\x14\n\x0crelease_date\x18\x03 \x01(\t\x12\x14\n\x0cprogram_name\x18\x04 \x01(\t\" \n\x12\x43\x61librationRequest\x12\n\n\x02id\x18\x01 \x01(\x05\"M\n\x13\x43\x61librationResponse\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x14\n\x0cis_succeeded\x18\x02 \x03(\x08\x12\x14\n\x0cindy_cam_rms\x18\x03 \x03(\x02\"$\n\x16\x43\x61librationDoneRequest\x12\n\n\x02id\x18\x01 \x01(\x05\"h\n\x17\x43\x61librationDoneResponse\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x15\n\ris_calibrated\x18\x02 \x01(\x08\x12\x14\n\x0cis_succeeded\x18\x03 \x03(\x08\x12\x14\n\x0cindy_cam_rms\x18\x04 \x03(\x02\"(\n\x0cImageRequest\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x0c\n\x04type\x18\x02 \x01(\x05\"\x90\x01\n\rImageResponse\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x0c\n\x04type\x18\x02 \x01(\x05\x12\r\n\x05width\x18\x03 \x01(\x05\x12\x0e\n\x06height\x18\x04 \x01(\x05\x12\r\n\x05\x63olor\x18\x05 \x01(\x0c\x12\r\n\x05\x64\x65pth\x18\x06 \x01(\x0c\x12\x13\n\x0b\x64\x65pth_scale\x18\x07 \x01(\x02\x12\x13\n\x0b\x65rror_state\x18\x08 \x01(\x08\"\x15\n\x07Request\x12\n\n\x02id\x18\x01 \x01(\x05\"A\n\tClassList\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x13\n\x0b\x63lass_names\x18\x02 \x03(\t\x12\x13\n\x0b\x65rror_state\x18\x08 \x01(\x08\"L\n\rDetectRequest\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x0b\n\x03\x63ls\x18\x02 \x01(\x05\x12\x10\n\x08pose_cmd\x18\x03 \x03(\x02\x12\x10\n\x08robot_ip\x18\x04 \x01(\t\"*\n\x0fRetrieveRequest\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x0b\n\x03\x63ls\x18\x02 \x01(\x05\"\xca\x01\n\x0e\x44\x65tectResponse\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x10\n\x08\x64\x65tected\x18\x02 \x01(\x08\x12\x0e\n\x06passed\x18\x03 \x01(\x08\x12\x0b\n\x03\x63ls\x18\x04 \x01(\x05\x12\x13\n\x0btar_ee_pose\x18\x05 \x03(\x02\x12\x15\n\rtar_tool_pose\x18\x06 \x03(\x02\x12\x14\n\x0ctar_obj_pose\x18\x07 \x03(\x02\x12\x10\n\x08tool_idx\x18\x08 \x01(\x05\x12\x13\n\x0b\x65rror_state\x18\t \x01(\x08\x12\x14\n\x0c\x65rror_module\x18\n \x01(\t2\xda\t\n\x07\x45yeTask\x12x\n\rGetServerInfo\x12\x31.IndyFramework.Protobuf.EyeTask.ServerInfoRequest\x1a\x32.IndyFramework.Protobuf.EyeTask.ServerInfoResponse\"\x00\x12i\n\x08GetImage\x12,.IndyFramework.Protobuf.EyeTask.ImageRequest\x1a-.IndyFramework.Protobuf.EyeTask.ImageResponse\"\x00\x12\x64\n\x0cGetClassList\x12\'.IndyFramework.Protobuf.EyeTask.Request\x1a).IndyFramework.Protobuf.EyeTask.ClassList\"\x00\x12i\n\x06\x44\x65tect\x12-.IndyFramework.Protobuf.EyeTask.DetectRequest\x1a..IndyFramework.Protobuf.EyeTask.DetectResponse\"\x00\x12m\n\x08Retrieve\x12/.IndyFramework.Protobuf.EyeTask.RetrieveRequest\x1a..IndyFramework.Protobuf.EyeTask.DetectResponse\"\x00\x12\x8a\x01\n\x13GetWeldingLinesInfo\x12\x37.IndyFramework.Protobuf.EyeTask.WeldingLinesInfoRequest\x1a\x38.IndyFramework.Protobuf.EyeTask.WeldingLinesInfoResponse\"\x00\x12\x8a\x01\n\x13GetStraightLineInfo\x12\x37.IndyFramework.Protobuf.EyeTask.StraightLineInfoRequest\x1a\x38.IndyFramework.Protobuf.EyeTask.StraightLineInfoResponse\"\x00\x12\x8a\x01\n\x13GetCircularLineInfo\x12\x37.IndyFramework.Protobuf.EyeTask.CircularLineInfoRequest\x1a\x38.IndyFramework.Protobuf.EyeTask.CircularLineInfoResponse\"\x00\x12z\n\rDoCalibration\x12\x32.IndyFramework.Protobuf.EyeTask.CalibrationRequest\x1a\x33.IndyFramework.Protobuf.EyeTask.CalibrationResponse\"\x00\x12\x86\x01\n\x11IsCalibrationDone\x12\x36.IndyFramework.Protobuf.EyeTask.CalibrationDoneRequest\x1a\x37.IndyFramework.Protobuf.EyeTask.CalibrationDoneResponse\"\x00\x42\x39\n io.grpc.custom.neuromeka.EyeTaskB\x0c\x45yeTaskProtoP\x01\xa2\x02\x04\x46\x44\x44Ob\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'indyeye_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'\n io.grpc.custom.neuromeka.EyeTaskB\014EyeTaskProtoP\001\242\002\004FDDO'
  _globals['_POINT']._serialized_start=49
  _globals['_POINT']._serialized_end=89
  _globals['_WELDINGLINESINFO']._serialized_start=92
  _globals['_WELDINGLINESINFO']._serialized_end=242
  _globals['_WELDINGLINESINFOREQUEST']._serialized_start=244
  _globals['_WELDINGLINESINFOREQUEST']._serialized_end=281
  _globals['_WELDINGLINESINFORESPONSE']._serialized_start=284
  _globals['_WELDINGLINESINFORESPONSE']._serialized_end=430
  _globals['_STRAIGHTLINEINFOREQUEST']._serialized_start=432
  _globals['_STRAIGHTLINEINFOREQUEST']._serialized_end=469
  _globals['_STRAIGHTLINEINFORESPONSE']._serialized_start=472
  _globals['_STRAIGHTLINEINFORESPONSE']._serialized_end=711
  _globals['_CIRCULARLINETHREEPOINTS']._serialized_start=714
  _globals['_CIRCULARLINETHREEPOINTS']._serialized_end=915
  _globals['_CIRCULARLINECENTERRADIUS']._serialized_start=917
  _globals['_CIRCULARLINECENTERRADIUS']._serialized_end=1020
  _globals['_CIRCULARLINEINFOREQUEST']._serialized_start=1022
  _globals['_CIRCULARLINEINFOREQUEST']._serialized_end=1059
  _globals['_CIRCULARLINEINFORESPONSE']._serialized_start=1062
  _globals['_CIRCULARLINEINFORESPONSE']._serialized_end=1408
  _globals['_SERVERINFOREQUEST']._serialized_start=1410
  _globals['_SERVERINFOREQUEST']._serialized_end=1441
  _globals['_SERVERINFORESPONSE']._serialized_start=1443
  _globals['_SERVERINFORESPONSE']._serialized_end=1536
  _globals['_CALIBRATIONREQUEST']._serialized_start=1538
  _globals['_CALIBRATIONREQUEST']._serialized_end=1570
  _globals['_CALIBRATIONRESPONSE']._serialized_start=1572
  _globals['_CALIBRATIONRESPONSE']._serialized_end=1649
  _globals['_CALIBRATIONDONEREQUEST']._serialized_start=1651
  _globals['_CALIBRATIONDONEREQUEST']._serialized_end=1687
  _globals['_CALIBRATIONDONERESPONSE']._serialized_start=1689
  _globals['_CALIBRATIONDONERESPONSE']._serialized_end=1793
  _globals['_IMAGEREQUEST']._serialized_start=1795
  _globals['_IMAGEREQUEST']._serialized_end=1835
  _globals['_IMAGERESPONSE']._serialized_start=1838
  _globals['_IMAGERESPONSE']._serialized_end=1982
  _globals['_REQUEST']._serialized_start=1984
  _globals['_REQUEST']._serialized_end=2005
  _globals['_CLASSLIST']._serialized_start=2007
  _globals['_CLASSLIST']._serialized_end=2072
  _globals['_DETECTREQUEST']._serialized_start=2074
  _globals['_DETECTREQUEST']._serialized_end=2150
  _globals['_RETRIEVEREQUEST']._serialized_start=2152
  _globals['_RETRIEVEREQUEST']._serialized_end=2194
  _globals['_DETECTRESPONSE']._serialized_start=2197
  _globals['_DETECTRESPONSE']._serialized_end=2399
  _globals['_EYETASK']._serialized_start=2402
  _globals['_EYETASK']._serialized_end=3644
# @@protoc_insertion_point(module_scope)
