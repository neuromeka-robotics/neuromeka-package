# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: config_msgs.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import common_msgs_pb2 as common__msgs__pb2
import device_msgs_pb2 as device__msgs__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x11\x63onfig_msgs.proto\x12\x12Nrmk.IndyFramework\x1a\x11\x63ommon_msgs.proto\x1a\x11\x64\x65vice_msgs.proto\"\x15\n\x05\x46rame\x12\x0c\n\x04\x66pos\x18\x01 \x03(\x02\"\x18\n\x08JointPos\x12\x0c\n\x04jpos\x18\x01 \x03(\x02\":\n\x0bPlanarFrame\x12\r\n\x05\x66pos0\x18\x01 \x03(\x02\x12\r\n\x05\x66pos1\x18\x02 \x03(\x02\x12\r\n\x05\x66pos2\x18\x03 \x03(\x02\"K\n\x0b\x46rameResult\x12\x0c\n\x04\x66pos\x18\x01 \x03(\x02\x12.\n\x08response\x18\x02 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.Response\"\x16\n\x05Ratio\x12\r\n\x05ratio\x18\x01 \x01(\r\"2\n\x12\x41utoServoOffConfig\x12\x0e\n\x06\x65nable\x18\x01 \x01(\x08\x12\x0c\n\x04time\x18\x02 \x01(\x02\"\xc4\x02\n\x10\x43ollTuningConfig\x12G\n\tprecision\x18\x01 \x01(\x0e\x32\x34.Nrmk.IndyFramework.CollTuningConfig.TuningPrecision\x12\x46\n\x0ctuning_space\x18\x02 \x01(\x0e\x32\x30.Nrmk.IndyFramework.CollTuningConfig.TuningSpace\x12\x15\n\rvel_level_max\x18\x03 \x01(\x05\"?\n\x0fTuningPrecision\x12\x0c\n\x08LOW_TUNE\x10\x00\x12\x0f\n\x0bMIDDLE_TUNE\x10\x01\x12\r\n\tHIGH_TUNE\x10\x02\"G\n\x0bTuningSpace\x12\x0b\n\x07NO_TUNE\x10\x00\x12\x0e\n\nJOINT_TUNE\x10\x01\x12\r\n\tTASK_TUNE\x10\x02\x12\x0c\n\x08\x41LL_TUNE\x10\x03\"3\n\x0cJointGainSet\x12\n\n\x02kp\x18\x01 \x03(\x02\x12\n\n\x02kv\x18\x02 \x03(\x02\x12\x0b\n\x03kl2\x18\x03 \x03(\x02\"2\n\x0bTaskGainSet\x12\n\n\x02kp\x18\x01 \x03(\x02\x12\n\n\x02kv\x18\x02 \x03(\x02\x12\x0b\n\x03kl2\x18\x03 \x03(\x02\"Q\n\x10ImpedanceGainSet\x12\x0c\n\x04mass\x18\x01 \x03(\x02\x12\x0f\n\x07\x64\x61mping\x18\x02 \x03(\x02\x12\x11\n\tstiffness\x18\x03 \x03(\x02\x12\x0b\n\x03kl2\x18\x04 \x03(\x02\"\x7f\n\x0c\x46orceGainSet\x12\n\n\x02kp\x18\x01 \x03(\x02\x12\n\n\x02kv\x18\x02 \x03(\x02\x12\x0b\n\x03kl2\x18\x03 \x03(\x02\x12\x0c\n\x04mass\x18\x04 \x03(\x02\x12\x0f\n\x07\x64\x61mping\x18\x05 \x03(\x02\x12\x11\n\tstiffness\x18\x06 \x03(\x02\x12\x0b\n\x03kpf\x18\x07 \x03(\x02\x12\x0b\n\x03kif\x18\x08 \x03(\x02\"i\n\x0bTestGainSet\x12\r\n\x05kpctc\x18\x01 \x03(\x02\x12\r\n\x05kvctc\x18\x02 \x03(\x02\x12\r\n\x05kictc\x18\x03 \x03(\x02\x12\r\n\x05knric\x18\x04 \x03(\x02\x12\x0e\n\x06kpnric\x18\x05 \x03(\x02\x12\x0e\n\x06kinric\x18\x06 \x03(\x02\"\xa5\x01\n\rCustomGainSet\x12\r\n\x05gain0\x18\x01 \x03(\x02\x12\r\n\x05gain1\x18\x02 \x03(\x02\x12\r\n\x05gain2\x18\x03 \x03(\x02\x12\r\n\x05gain3\x18\x04 \x03(\x02\x12\r\n\x05gain4\x18\x05 \x03(\x02\x12\r\n\x05gain5\x18\x06 \x03(\x02\x12\r\n\x05gain6\x18\x07 \x03(\x02\x12\r\n\x05gain7\x18\x08 \x03(\x02\x12\r\n\x05gain8\x18\t \x03(\x02\x12\r\n\x05gain9\x18\n \x03(\x02\":\n\x16NewControllerTestState\x12\x0f\n\x07Jenable\x18\x01 \x01(\x08\x12\x0f\n\x07Tenable\x18\x02 \x01(\x08\"\x87\x01\n\x0f\x46rictionCompSet\x12\x1b\n\x13\x63ontrol_comp_enable\x18\x01 \x01(\x08\x12\x1b\n\x13\x63ontrol_comp_levels\x18\x02 \x03(\x05\x12\x1c\n\x14teaching_comp_enable\x18\x03 \x01(\x08\x12\x1c\n\x14teaching_comp_levels\x18\x04 \x03(\x05\"(\n\x0eMountingAngles\x12\n\n\x02ry\x18\x01 \x01(\x02\x12\n\n\x02rz\x18\x02 \x01(\x02\"G\n\x0eToolProperties\x12\x0c\n\x04mass\x18\x01 \x01(\x02\x12\x16\n\x0e\x63\x65nter_of_mass\x18\x02 \x03(\x02\x12\x0f\n\x07inertia\x18\x03 \x03(\x02\"#\n\x12\x43ollisionSensLevel\x12\r\n\x05level\x18\x01 \x01(\r\"\xb2\x02\n\x13\x43ollisionThresholds\x12\x16\n\x0ej_torque_bases\x18\x01 \x03(\x02\x12\x19\n\x11j_torque_tangents\x18\x02 \x03(\x02\x12\x16\n\x0et_torque_bases\x18\x03 \x03(\x02\x12\x19\n\x11t_torque_tangents\x18\x04 \x03(\x02\x12\x13\n\x0b\x65rror_bases\x18\x05 \x03(\x02\x12\x16\n\x0e\x65rror_tangents\x18\x06 \x03(\x02\x12\x1f\n\x17t_constvel_torque_bases\x18\x07 \x03(\x02\x12\"\n\x1at_constvel_torque_tangents\x18\x08 \x03(\x02\x12\x1f\n\x17t_conveyor_torque_bases\x18\t \x03(\x02\x12\"\n\x1at_conveyor_torque_tangents\x18\n \x03(\x02\"t\n\x0f\x43ollisionPolicy\x12\x37\n\x06policy\x18\x01 \x01(\x0e\x32\'.Nrmk.IndyFramework.CollisionPolicyType\x12\x12\n\nsleep_time\x18\x02 \x01(\x02\x12\x14\n\x0cgravity_time\x18\x03 \x01(\x02\"\xe6\x01\n\x0cSafetyLimits\x12\x13\n\x0bpower_limit\x18\x01 \x01(\x02\x12\x19\n\x11power_limit_ratio\x18\x02 \x01(\x02\x12\x17\n\x0ftcp_force_limit\x18\x03 \x01(\x02\x12\x1d\n\x15tcp_force_limit_ratio\x18\x04 \x01(\x02\x12\x17\n\x0ftcp_speed_limit\x18\x05 \x01(\x02\x12\x1d\n\x15tcp_speed_limit_ratio\x18\x06 \x01(\x02\x12\x1a\n\x12joint_upper_limits\x18\x07 \x03(\x02\x12\x1a\n\x12joint_lower_limits\x18\x08 \x03(\x02\"\xb0\x03\n\x10SafetyStopConfig\x12G\n\x1djoint_position_limit_stop_cat\x18\x01 \x01(\x0e\x32 .Nrmk.IndyFramework.StopCategory\x12\x44\n\x1ajoint_speed_limit_stop_cat\x18\x02 \x01(\x0e\x32 .Nrmk.IndyFramework.StopCategory\x12\x45\n\x1bjoint_torque_limit_stop_cat\x18\x03 \x01(\x0e\x32 .Nrmk.IndyFramework.StopCategory\x12\x42\n\x18tcp_speed_limit_stop_cat\x18\x04 \x01(\x0e\x32 .Nrmk.IndyFramework.StopCategory\x12\x42\n\x18tcp_force_limit_stop_cat\x18\x05 \x01(\x0e\x32 .Nrmk.IndyFramework.StopCategory\x12>\n\x14power_limit_stop_cat\x18\x06 \x01(\x0e\x32 .Nrmk.IndyFramework.StopCategory\"\xe9\x01\n\x08\x44IConfig\x12\x15\n\rfunction_code\x18\x01 \x01(\x05\x12\x15\n\rfunction_name\x18\x02 \x01(\t\x12\x39\n\x0etriggerSignals\x18\x03 \x03(\x0b\x32!.Nrmk.IndyFramework.DigitalSignal\x12\x39\n\x0esuccessSignals\x18\x04 \x03(\x0b\x32!.Nrmk.IndyFramework.DigitalSignal\x12\x39\n\x0e\x66\x61ilureSignals\x18\x05 \x03(\x0b\x32!.Nrmk.IndyFramework.DigitalSignal\"@\n\x0c\x44IConfigList\x12\x30\n\ndi_configs\x18\x01 \x03(\x0b\x32\x1c.Nrmk.IndyFramework.DIConfig\"\x9f\x01\n\x08\x44OConfig\x12\x12\n\nstate_code\x18\x01 \x01(\x05\x12\x12\n\nstate_name\x18\x02 \x01(\t\x12\x34\n\tonSignals\x18\x03 \x03(\x0b\x32!.Nrmk.IndyFramework.DigitalSignal\x12\x35\n\noffSignals\x18\x04 \x03(\x0b\x32!.Nrmk.IndyFramework.DigitalSignal\"@\n\x0c\x44OConfigList\x12\x30\n\ndo_configs\x18\x01 \x03(\x0b\x32\x1c.Nrmk.IndyFramework.DOConfig\"0\n\x12GetReducedRatioRes\x12\r\n\x05ratio\x18\x01 \x01(\x02\x12\x0b\n\x03msg\x18\x64 \x01(\t\"0\n\x12GetReducedSpeedRes\x12\r\n\x05speed\x18\x01 \x01(\x02\x12\x0b\n\x03msg\x18\x64 \x01(\t\"#\n\x12SetReducedSpeedReq\x12\r\n\x05speed\x18\x01 \x01(\x02\"\xe1\x04\n\x0e\x46TSensorDevice\x12G\n\x08\x64\x65v_type\x18\x01 \x01(\x0e\x32\x35.Nrmk.IndyFramework.FTSensorDevice.FTSensorDeviceType\x12J\n\x08\x63om_type\x18\x02 \x01(\x0e\x32\x38.Nrmk.IndyFramework.FTSensorDevice.FTSensorDeviceComType\x12\x12\n\nip_address\x18\x03 \x01(\t\x12%\n\x1d\x66t_frame_translation_offset_x\x18\x0b \x01(\x02\x12%\n\x1d\x66t_frame_translation_offset_y\x18\x0c \x01(\x02\x12%\n\x1d\x66t_frame_translation_offset_z\x18\r \x01(\x02\x12\"\n\x1a\x66t_frame_rotation_offset_r\x18\x0e \x01(\x02\x12\"\n\x1a\x66t_frame_rotation_offset_p\x18\x0f \x01(\x02\x12\"\n\x1a\x66t_frame_rotation_offset_y\x18\x10 \x01(\x02\"q\n\x12\x46TSensorDeviceType\x12\x08\n\x04NONE\x10\x00\x12\x0e\n\nAFT200_D80\x10\x01\x12\x11\n\rAFT200_D80_EC\x10\x02\x12\x0e\n\nRFT80_6A01\x10\x03\x12\x0e\n\nRFT60_HA01\x10\x04\x12\x0e\n\nHEX_E_H_QC\x10\x05\"R\n\x15\x46TSensorDeviceComType\x12\x0e\n\nENDTOOLCAN\x10\x00\x12\t\n\x05\x43\x42\x43\x41N\x10\x01\x12\x10\n\x0cMODBUSCLIENT\x10\x02\x12\x0c\n\x08\x45THERCAT\x10\x03\" \n\x11\x46TSensorDeviceRes\x12\x0b\n\x03msg\x18\x64 \x01(\t\"N\n\x0cTeleOpParams\x12\x15\n\rsmooth_factor\x18\x01 \x01(\x02\x12\x13\n\x0b\x63utoff_freq\x18\x02 \x01(\x02\x12\x12\n\nerror_gain\x18\x03 \x01(\x02\"\x99\x02\n\x10KinematicsParams\x12\x35\n\x03mdh\x18\x01 \x03(\x0b\x32(.Nrmk.IndyFramework.KinematicsParams.MDH\x1a\x98\x01\n\x03MDH\x12\t\n\x01\x61\x18\x01 \x01(\x02\x12\r\n\x05\x61lpha\x18\x02 \x01(\x02\x12\n\n\x02\x64\x30\x18\x03 \x01(\x02\x12\x0e\n\x06theta0\x18\x04 \x01(\x02\x12<\n\x04type\x18\x05 \x01(\x0e\x32..Nrmk.IndyFramework.KinematicsParams.JointType\x12\r\n\x05index\x18\n \x01(\x05\x12\x0e\n\x06parent\x18\x0b \x01(\x05\"3\n\tJointType\x12\x0c\n\x08REVOLUTE\x10\x00\x12\r\n\tPRISMATIC\x10\x01\x12\t\n\x05\x46IXED\x10\x02\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'config_msgs_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_FRAME']._serialized_start=79
  _globals['_FRAME']._serialized_end=100
  _globals['_JOINTPOS']._serialized_start=102
  _globals['_JOINTPOS']._serialized_end=126
  _globals['_PLANARFRAME']._serialized_start=128
  _globals['_PLANARFRAME']._serialized_end=186
  _globals['_FRAMERESULT']._serialized_start=188
  _globals['_FRAMERESULT']._serialized_end=263
  _globals['_RATIO']._serialized_start=265
  _globals['_RATIO']._serialized_end=287
  _globals['_AUTOSERVOOFFCONFIG']._serialized_start=289
  _globals['_AUTOSERVOOFFCONFIG']._serialized_end=339
  _globals['_COLLTUNINGCONFIG']._serialized_start=342
  _globals['_COLLTUNINGCONFIG']._serialized_end=666
  _globals['_COLLTUNINGCONFIG_TUNINGPRECISION']._serialized_start=530
  _globals['_COLLTUNINGCONFIG_TUNINGPRECISION']._serialized_end=593
  _globals['_COLLTUNINGCONFIG_TUNINGSPACE']._serialized_start=595
  _globals['_COLLTUNINGCONFIG_TUNINGSPACE']._serialized_end=666
  _globals['_JOINTGAINSET']._serialized_start=668
  _globals['_JOINTGAINSET']._serialized_end=719
  _globals['_TASKGAINSET']._serialized_start=721
  _globals['_TASKGAINSET']._serialized_end=771
  _globals['_IMPEDANCEGAINSET']._serialized_start=773
  _globals['_IMPEDANCEGAINSET']._serialized_end=854
  _globals['_FORCEGAINSET']._serialized_start=856
  _globals['_FORCEGAINSET']._serialized_end=983
  _globals['_TESTGAINSET']._serialized_start=985
  _globals['_TESTGAINSET']._serialized_end=1090
  _globals['_CUSTOMGAINSET']._serialized_start=1093
  _globals['_CUSTOMGAINSET']._serialized_end=1258
  _globals['_NEWCONTROLLERTESTSTATE']._serialized_start=1260
  _globals['_NEWCONTROLLERTESTSTATE']._serialized_end=1318
  _globals['_FRICTIONCOMPSET']._serialized_start=1321
  _globals['_FRICTIONCOMPSET']._serialized_end=1456
  _globals['_MOUNTINGANGLES']._serialized_start=1458
  _globals['_MOUNTINGANGLES']._serialized_end=1498
  _globals['_TOOLPROPERTIES']._serialized_start=1500
  _globals['_TOOLPROPERTIES']._serialized_end=1571
  _globals['_COLLISIONSENSLEVEL']._serialized_start=1573
  _globals['_COLLISIONSENSLEVEL']._serialized_end=1608
  _globals['_COLLISIONTHRESHOLDS']._serialized_start=1611
  _globals['_COLLISIONTHRESHOLDS']._serialized_end=1917
  _globals['_COLLISIONPOLICY']._serialized_start=1919
  _globals['_COLLISIONPOLICY']._serialized_end=2035
  _globals['_SAFETYLIMITS']._serialized_start=2038
  _globals['_SAFETYLIMITS']._serialized_end=2268
  _globals['_SAFETYSTOPCONFIG']._serialized_start=2271
  _globals['_SAFETYSTOPCONFIG']._serialized_end=2703
  _globals['_DICONFIG']._serialized_start=2706
  _globals['_DICONFIG']._serialized_end=2939
  _globals['_DICONFIGLIST']._serialized_start=2941
  _globals['_DICONFIGLIST']._serialized_end=3005
  _globals['_DOCONFIG']._serialized_start=3008
  _globals['_DOCONFIG']._serialized_end=3167
  _globals['_DOCONFIGLIST']._serialized_start=3169
  _globals['_DOCONFIGLIST']._serialized_end=3233
  _globals['_GETREDUCEDRATIORES']._serialized_start=3235
  _globals['_GETREDUCEDRATIORES']._serialized_end=3283
  _globals['_GETREDUCEDSPEEDRES']._serialized_start=3285
  _globals['_GETREDUCEDSPEEDRES']._serialized_end=3333
  _globals['_SETREDUCEDSPEEDREQ']._serialized_start=3335
  _globals['_SETREDUCEDSPEEDREQ']._serialized_end=3370
  _globals['_FTSENSORDEVICE']._serialized_start=3373
  _globals['_FTSENSORDEVICE']._serialized_end=3982
  _globals['_FTSENSORDEVICE_FTSENSORDEVICETYPE']._serialized_start=3785
  _globals['_FTSENSORDEVICE_FTSENSORDEVICETYPE']._serialized_end=3898
  _globals['_FTSENSORDEVICE_FTSENSORDEVICECOMTYPE']._serialized_start=3900
  _globals['_FTSENSORDEVICE_FTSENSORDEVICECOMTYPE']._serialized_end=3982
  _globals['_FTSENSORDEVICERES']._serialized_start=3984
  _globals['_FTSENSORDEVICERES']._serialized_end=4016
  _globals['_TELEOPPARAMS']._serialized_start=4018
  _globals['_TELEOPPARAMS']._serialized_end=4096
  _globals['_KINEMATICSPARAMS']._serialized_start=4099
  _globals['_KINEMATICSPARAMS']._serialized_end=4380
  _globals['_KINEMATICSPARAMS_MDH']._serialized_start=4175
  _globals['_KINEMATICSPARAMS_MDH']._serialized_end=4327
  _globals['_KINEMATICSPARAMS_JOINTTYPE']._serialized_start=4329
  _globals['_KINEMATICSPARAMS_JOINTTYPE']._serialized_end=4380
# @@protoc_insertion_point(module_scope)
