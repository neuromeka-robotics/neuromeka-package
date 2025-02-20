# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ethercat_msgs.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x13\x65thercat_msgs.proto\x12\x12Nrmk.IndyFramework\"\x1e\n\x0cMasterStatus\x12\x0e\n\x06status\x18\x01 \x01(\r\"\x1d\n\x0bSlaveStatus\x12\x0e\n\x06status\x18\x01 \x03(\r\"\"\n\x10\x45\x63\x61tDomainStatus\x12\x0e\n\x06status\x18\x01 \x01(\r\" \n\x0f\x45\x63\x61tSystemReady\x12\r\n\x05ready\x18\x01 \x03(\x08\"\x1c\n\x0b\x45\x63\x61tServoOn\x12\r\n\x05servo\x18\x01 \x03(\x08\"\\\n\x0cSlaveTypeNum\x12\x11\n\tnum_servo\x18\x01 \x01(\r\x12\x13\n\x0bnum_ioboard\x18\x02 \x01(\r\x12\x13\n\x0bnum_endtool\x18\x03 \x01(\r\x12\x0f\n\x07num_dio\x18\x04 \x01(\r\" \n\nServoIndex\x12\x12\n\nservoIndex\x18\x01 \x01(\r\"\x1e\n\tEcatIndex\x12\x11\n\tecatIndex\x18\x01 \x01(\r\"w\n\x07ServoTx\x12\x12\n\nstatusWord\x18\x01 \x01(\r\x12\x12\n\nmodeOpDisp\x18\x02 \x01(\x05\x12\x16\n\x0e\x61\x63tualPosition\x18\x03 \x01(\x05\x12\x16\n\x0e\x61\x63tualVelocity\x18\x04 \x01(\x05\x12\x14\n\x0c\x61\x63tualTorque\x18\x05 \x01(\x05\"t\n\x07ServoRx\x12\x13\n\x0b\x63ontrolWord\x18\x01 \x01(\r\x12\x0e\n\x06modeOp\x18\x02 \x01(\x05\x12\x16\n\x0etargetPosition\x18\x03 \x01(\x03\x12\x16\n\x0etargetVelocity\x18\x04 \x01(\x05\x12\x14\n\x0ctargetTorque\x18\x05 \x01(\x05\"K\n\x0cServoTxIndex\x12\x12\n\nservoIndex\x18\x01 \x01(\r\x12\'\n\x02tx\x18\x02 \x01(\x0b\x32\x1b.Nrmk.IndyFramework.ServoTx\"K\n\x0cServoRxIndex\x12\x12\n\nservoIndex\x18\x01 \x01(\r\x12\'\n\x02rx\x18\x02 \x01(\x0b\x32\x1b.Nrmk.IndyFramework.ServoRx\"\xdf\x01\n\x0bServoTxKeba\x12\x12\n\nstatusWord\x18\x01 \x01(\r\x12\x13\n\x0bstatusWord2\x18\x02 \x01(\r\x12\x13\n\x0bstatusWord3\x18\x03 \x01(\r\x12\x16\n\x0e\x61\x63tualPosition\x18\x04 \x01(\x05\x12\x17\n\x0f\x61\x63tualPosition2\x18\x05 \x01(\x05\x12\x17\n\x0f\x61\x63tualPosition3\x18\x06 \x01(\x05\x12\x16\n\x0e\x61\x63tualVelocity\x18\x07 \x01(\x05\x12\x17\n\x0f\x61\x63tualVelocity2\x18\x08 \x01(\x05\x12\x17\n\x0f\x61\x63tualVelocity3\x18\t \x01(\x05\"\xdc\x01\n\x0bServoRxKeba\x12\x13\n\x0b\x63ontrolWord\x18\x01 \x01(\r\x12\x14\n\x0c\x63ontrolWord2\x18\x02 \x01(\r\x12\x14\n\x0c\x63ontrolWord3\x18\x03 \x01(\r\x12\x16\n\x0etargetPosition\x18\x04 \x01(\x03\x12\x17\n\x0ftargetPosition2\x18\x05 \x01(\x03\x12\x17\n\x0ftargetPosition3\x18\x06 \x01(\x03\x12\x14\n\x0ctargetTorque\x18\x07 \x01(\x05\x12\x15\n\rtargetTorque2\x18\x08 \x01(\x05\x12\x15\n\rtargetTorque3\x18\t \x01(\x05\"S\n\x10ServoRxIndexKeba\x12\x12\n\nservoIndex\x18\x01 \x01(\r\x12+\n\x02rx\x18\x02 \x01(\x0b\x32\x1f.Nrmk.IndyFramework.ServoRxKeba\" \n\tServoTemp\x12\x13\n\x0btemperature\x18\x01 \x01(\x02\"\x1f\n\nServoError\x12\x11\n\terrorCode\x18\x01 \x01(\r\".\n\nServoBrake\x12\x11\n\tecatIndex\x18\x01 \x01(\r\x12\r\n\x05onoff\x18\x02 \x01(\x08\"R\n\x08\x46TSensor\x12\n\n\x02\x66x\x18\x01 \x01(\x05\x12\n\n\x02\x66y\x18\x02 \x01(\x05\x12\n\n\x02\x66z\x18\x03 \x01(\x05\x12\n\n\x02tx\x18\x04 \x01(\x05\x12\n\n\x02ty\x18\x05 \x01(\x05\x12\n\n\x02tz\x18\x06 \x01(\x05\"\x80\x01\n\tEndtoolTx\x12\x0e\n\x06status\x18\x01 \x01(\r\x12\x0e\n\x06\x62utton\x18\x02 \x01(\r\x12/\n\tft_sensor\x18\x03 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.FTSensor\x12\x10\n\x08\x66t_state\x18\x04 \x01(\r\x12\x10\n\x08\x66t_error\x18\x05 \x01(\r\"z\n\tEndtoolRx\x12\x0b\n\x03\x65qc\x18\x01 \x01(\r\x12\x0f\n\x07gripper\x18\x02 \x01(\r\x12\x10\n\x08\x66t_param\x18\x03 \x01(\r\x12\x10\n\x08led_mode\x18\x04 \x01(\r\x12\r\n\x05led_g\x18\x05 \x01(\r\x12\r\n\x05led_r\x18\x06 \x01(\r\x12\r\n\x05led_b\x18\x07 \x01(\r\"\x8c\x03\n\x10\x45ndtoolDockingTx\x12\x16\n\x0e\x64ocking_status\x18\x01 \x01(\r\x12\x16\n\x0e\x64ocking_button\x18\x02 \x01(\r\x12\x15\n\rdocking_dist1\x18\x03 \x01(\r\x12\x15\n\rdocking_dist2\x18\x04 \x01(\r\x12\x15\n\rdocking_dist3\x18\x05 \x01(\r\x12\x15\n\rdocking_dist4\x18\x06 \x01(\r\x12\x15\n\rdocking_dist5\x18\x07 \x01(\r\x12\x15\n\rdocking_dist6\x18\x08 \x01(\r\x12\x15\n\rdocking_dist7\x18\t \x01(\r\x12\x15\n\rdocking_dist8\x18\n \x01(\r\x12\x15\n\rdirect_status\x18\x0b \x01(\r\x12\x15\n\rdirect_button\x18\x0c \x01(\r\x12\x16\n\x0e\x64irect_encoder\x18\r \x01(\r\x12\x11\n\tjoystic1x\x18\x0e \x01(\r\x12\x11\n\tjoystic1y\x18\x0f \x01(\r\x12\x11\n\tjoystic2x\x18\x10 \x01(\r\x12\x11\n\tjoystic2y\x18\x11 \x01(\r\"\x19\n\x06LedDim\x12\x0f\n\x07led_dim\x18\x01 \x01(\r\"\x96\x01\n\x0eSRKeyEndtoolRx\x12\x0c\n\x04\x64out\x18\x01 \x01(\r\x12\x0f\n\x07tool_Id\x18\x02 \x01(\r\x12\x10\n\x08set_Tool\x18\x03 \x01(\r\x12\x1a\n\x12tool_Closing_Force\x18\x04 \x01(\r\x12\x1a\n\x12tool_Opening_Force\x18\x05 \x01(\r\x12\x1b\n\x13tool_Force_Location\x18\x06 \x01(\r\"|\n\x0eSRKeyEndtoolTx\x12\x0b\n\x03\x64in\x18\x01 \x01(\r\x12\x13\n\x0btool_Status\x18\x02 \x01(\r\x12\x15\n\rtool_Location\x18\x03 \x01(\r\x12\x0f\n\x07\x61nalog0\x18\x04 \x01(\r\x12\x0f\n\x07\x61nalog1\x18\x05 \x01(\r\x12\x0f\n\x07version\x18\x06 \x01(\r\"\x84\x01\n\tIOBoardTx\x12\x0c\n\x04\x64i5v\x18\x01 \x01(\r\x12\x0e\n\x06\x64i24v1\x18\x02 \x01(\r\x12\x0e\n\x06\x64i24v2\x18\x03 \x01(\r\x12\x0b\n\x03\x61i1\x18\x04 \x01(\r\x12\x0b\n\x03\x61i2\x18\x05 \x01(\r\x12/\n\tft_sensor\x18\x06 \x01(\x0b\x32\x1c.Nrmk.IndyFramework.FTSensor\"e\n\tIOBoardRx\x12\x0c\n\x04\x64o5v\x18\x01 \x01(\r\x12\x0e\n\x06\x64o24v1\x18\x02 \x01(\r\x12\x0e\n\x06\x64o24v2\x18\x03 \x01(\r\x12\x0b\n\x03\x61o1\x18\x04 \x01(\r\x12\x0b\n\x03\x61o2\x18\x05 \x01(\r\x12\x10\n\x08\x66t_param\x18\x06 \x01(\r\"\x1c\n\x08\x44IOIndex\x12\x10\n\x08\x64ioIndex\x18\x01 \x01(\r\"4\n\x0f\x44IODigitalInput\x12\x10\n\x08\x64ioIndex\x18\x01 \x01(\r\x12\x0f\n\x07\x64i_list\x18\x02 \x03(\r\"5\n\x10\x44IODigitalOutput\x12\x10\n\x08\x64ioIndex\x18\x01 \x01(\r\x12\x0f\n\x07\x64o_list\x18\x02 \x03(\r\"+\n\nServoParam\x12\x10\n\x08slaveIdx\x18\x01 \x01(\r\x12\x0b\n\x03val\x18\x02 \x01(\x05\"\x18\n\tSDOIntVal\x12\x0b\n\x03val\x18\x01 \x01(\x05\"\x1a\n\x0bSDOFloatVal\x12\x0b\n\x03val\x18\x01 \x01(\x02\"\x18\n\tSDOStrVal\x12\x0b\n\x03val\x18\x01 \x01(\t\"\x0e\n\x0c\x45therCATInfo\"\x1f\n\x0eRobotZeroCount\x12\r\n\x05\x63ount\x18\x01 \x01(\x05\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'ethercat_msgs_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_MASTERSTATUS']._serialized_start=43
  _globals['_MASTERSTATUS']._serialized_end=73
  _globals['_SLAVESTATUS']._serialized_start=75
  _globals['_SLAVESTATUS']._serialized_end=104
  _globals['_ECATDOMAINSTATUS']._serialized_start=106
  _globals['_ECATDOMAINSTATUS']._serialized_end=140
  _globals['_ECATSYSTEMREADY']._serialized_start=142
  _globals['_ECATSYSTEMREADY']._serialized_end=174
  _globals['_ECATSERVOON']._serialized_start=176
  _globals['_ECATSERVOON']._serialized_end=204
  _globals['_SLAVETYPENUM']._serialized_start=206
  _globals['_SLAVETYPENUM']._serialized_end=298
  _globals['_SERVOINDEX']._serialized_start=300
  _globals['_SERVOINDEX']._serialized_end=332
  _globals['_ECATINDEX']._serialized_start=334
  _globals['_ECATINDEX']._serialized_end=364
  _globals['_SERVOTX']._serialized_start=366
  _globals['_SERVOTX']._serialized_end=485
  _globals['_SERVORX']._serialized_start=487
  _globals['_SERVORX']._serialized_end=603
  _globals['_SERVOTXINDEX']._serialized_start=605
  _globals['_SERVOTXINDEX']._serialized_end=680
  _globals['_SERVORXINDEX']._serialized_start=682
  _globals['_SERVORXINDEX']._serialized_end=757
  _globals['_SERVOTXKEBA']._serialized_start=760
  _globals['_SERVOTXKEBA']._serialized_end=983
  _globals['_SERVORXKEBA']._serialized_start=986
  _globals['_SERVORXKEBA']._serialized_end=1206
  _globals['_SERVORXINDEXKEBA']._serialized_start=1208
  _globals['_SERVORXINDEXKEBA']._serialized_end=1291
  _globals['_SERVOTEMP']._serialized_start=1293
  _globals['_SERVOTEMP']._serialized_end=1325
  _globals['_SERVOERROR']._serialized_start=1327
  _globals['_SERVOERROR']._serialized_end=1358
  _globals['_SERVOBRAKE']._serialized_start=1360
  _globals['_SERVOBRAKE']._serialized_end=1406
  _globals['_FTSENSOR']._serialized_start=1408
  _globals['_FTSENSOR']._serialized_end=1490
  _globals['_ENDTOOLTX']._serialized_start=1493
  _globals['_ENDTOOLTX']._serialized_end=1621
  _globals['_ENDTOOLRX']._serialized_start=1623
  _globals['_ENDTOOLRX']._serialized_end=1745
  _globals['_ENDTOOLDOCKINGTX']._serialized_start=1748
  _globals['_ENDTOOLDOCKINGTX']._serialized_end=2144
  _globals['_LEDDIM']._serialized_start=2146
  _globals['_LEDDIM']._serialized_end=2171
  _globals['_SRKEYENDTOOLRX']._serialized_start=2174
  _globals['_SRKEYENDTOOLRX']._serialized_end=2324
  _globals['_SRKEYENDTOOLTX']._serialized_start=2326
  _globals['_SRKEYENDTOOLTX']._serialized_end=2450
  _globals['_IOBOARDTX']._serialized_start=2453
  _globals['_IOBOARDTX']._serialized_end=2585
  _globals['_IOBOARDRX']._serialized_start=2587
  _globals['_IOBOARDRX']._serialized_end=2688
  _globals['_DIOINDEX']._serialized_start=2690
  _globals['_DIOINDEX']._serialized_end=2718
  _globals['_DIODIGITALINPUT']._serialized_start=2720
  _globals['_DIODIGITALINPUT']._serialized_end=2772
  _globals['_DIODIGITALOUTPUT']._serialized_start=2774
  _globals['_DIODIGITALOUTPUT']._serialized_end=2827
  _globals['_SERVOPARAM']._serialized_start=2829
  _globals['_SERVOPARAM']._serialized_end=2872
  _globals['_SDOINTVAL']._serialized_start=2874
  _globals['_SDOINTVAL']._serialized_end=2898
  _globals['_SDOFLOATVAL']._serialized_start=2900
  _globals['_SDOFLOATVAL']._serialized_end=2926
  _globals['_SDOSTRVAL']._serialized_start=2928
  _globals['_SDOSTRVAL']._serialized_end=2952
  _globals['_ETHERCATINFO']._serialized_start=2954
  _globals['_ETHERCATINFO']._serialized_end=2968
  _globals['_ROBOTZEROCOUNT']._serialized_start=2970
  _globals['_ROBOTZEROCOUNT']._serialized_end=3001
# @@protoc_insertion_point(module_scope)
