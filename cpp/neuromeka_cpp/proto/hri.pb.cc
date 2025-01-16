// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: hri.proto

#include "hri.pb.h"

#include <algorithm>
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/extension_set.h"
#include "google/protobuf/wire_format_lite.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/generated_message_reflection.h"
#include "google/protobuf/reflection_ops.h"
#include "google/protobuf/wire_format.h"
#include "google/protobuf/generated_message_tctable_impl.h"
// @@protoc_insertion_point(includes)

// Must be included last.
#include "google/protobuf/port_def.inc"
PROTOBUF_PRAGMA_INIT_SEG
namespace _pb = ::google::protobuf;
namespace _pbi = ::google::protobuf::internal;
namespace _fl = ::google::protobuf::internal::field_layout;
namespace IndyFramework {
namespace Protobuf {
namespace HRI {
}  // namespace HRI
}  // namespace Protobuf
}  // namespace IndyFramework
static constexpr const ::_pb::EnumDescriptor**
    file_level_enum_descriptors_hri_2eproto = nullptr;
static constexpr const ::_pb::ServiceDescriptor**
    file_level_service_descriptors_hri_2eproto = nullptr;
const ::uint32_t TableStruct_hri_2eproto::offsets[1] = {};
static constexpr ::_pbi::MigrationSchema* schemas = nullptr;
static constexpr ::_pb::Message* const* file_default_instances = nullptr;
const char descriptor_table_protodef_hri_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
    "\n\thri.proto\022\032IndyFramework.Protobuf.HRI\032"
    "\016hri_msgs.proto2\365\250\001\n\003HRI\022a\n\tContyInit\022(."
    "IndyFramework.Protobuf.HRI.ContyInitReq\032"
    "(.IndyFramework.Protobuf.HRI.ContyInitRe"
    "s\"\000\022Y\n\013RobotUpdate\022!.IndyFramework.Proto"
    "buf.HRI.Empty\032%.IndyFramework.Protobuf.H"
    "RI.RobotData\"\000\022]\n\rProgramUpdate\022!.IndyFr"
    "amework.Protobuf.HRI.Empty\032\'.IndyFramewo"
    "rk.Protobuf.HRI.ProgramData\"\000\022]\n\rControl"
    "Update\022!.IndyFramework.Protobuf.HRI.Empt"
    "y\032\'.IndyFramework.Protobuf.HRI.ControlDa"
    "ta\"\000\022[\n\014LinearUpdate\022!.IndyFramework.Pro"
    "tobuf.HRI.Empty\032&.IndyFramework.Protobuf"
    ".HRI.LinearData\"\000\022^\n\010JogJoint\022\'.IndyFram"
    "ework.Protobuf.HRI.JogJointReq\032\'.IndyFra"
    "mework.Protobuf.HRI.JogJointRes\"\000\022[\n\007Jog"
    "Task\022&.IndyFramework.Protobuf.HRI.JogTas"
    "kReq\032&.IndyFramework.Protobuf.HRI.JogTas"
    "kRes\"\000\022[\n\007JogAxis\022&.IndyFramework.Protob"
    "uf.HRI.JogAxisReq\032&.IndyFramework.Protob"
    "uf.HRI.JogAxisRes\"\000\022X\n\013HoldToMoveJ\022$.Ind"
    "yFramework.Protobuf.HRI.MoveJReq\032!.IndyF"
    "ramework.Protobuf.HRI.Empty\"\000\022X\n\013HoldToM"
    "oveL\022$.IndyFramework.Protobuf.HRI.MoveLR"
    "eq\032!.IndyFramework.Protobuf.HRI.Empty\"\000\022"
    "X\n\013HoldToAxisJ\022$.IndyFramework.Protobuf."
    "HRI.AxisJReq\032!.IndyFramework.Protobuf.HR"
    "I.Empty\"\000\022d\n\nStopMotion\022).IndyFramework."
    "Protobuf.HRI.StopMotionReq\032).IndyFramewo"
    "rk.Protobuf.HRI.StopMotionRes\"\000\022g\n\013SetRe"
    "fFrame\022*.IndyFramework.Protobuf.HRI.SetR"
    "efFrameReq\032*.IndyFramework.Protobuf.HRI."
    "SetRefFrameRes\"\000\022y\n\021SetRefFramePlanar\0220."
    "IndyFramework.Protobuf.HRI.SetRefFramePl"
    "anarReq\0320.IndyFramework.Protobuf.HRI.Set"
    "RefFramePlanarRes\"\000\022j\n\014SetToolFrame\022+.In"
    "dyFramework.Protobuf.HRI.SetToolFrameReq"
    "\032+.IndyFramework.Protobuf.HRI.SetToolFra"
    "meRes\"\000\022m\n\rSetSpeedRatio\022,.IndyFramework"
    ".Protobuf.HRI.SetSpeedRatioReq\032,.IndyFra"
    "mework.Protobuf.HRI.SetSpeedRatioRes\"\000\022y"
    "\n\021SetSimulationMode\0220.IndyFramework.Prot"
    "obuf.HRI.SetSimulationModeReq\0320.IndyFram"
    "ework.Protobuf.HRI.SetSimulationModeRes\""
    "\000\022\205\001\n\025SetDirectTeachingMode\0224.IndyFramew"
    "ork.Protobuf.HRI.SetDirectTeachingModeRe"
    "q\0324.IndyFramework.Protobuf.HRI.SetDirect"
    "TeachingModeRes\"\000\022c\n\024SetCustomControlMod"
    "e\022#.IndyFramework.Protobuf.HRI.IntMode\032$"
    ".IndyFramework.Protobuf.HRI.Response\"\000\022`"
    "\n\024GetCustomControlMode\022!.IndyFramework.P"
    "rotobuf.HRI.Empty\032#.IndyFramework.Protob"
    "uf.HRI.IntMode\"\000\022\227\001\n\033SetSensorlessCompli"
    "anceMode\022:.IndyFramework.Protobuf.HRI.Se"
    "tSensorlessComplianceModeReq\032:.IndyFrame"
    "work.Protobuf.HRI.SetSensorlessComplianc"
    "eModeRes\"\000\022j\n\024GetBrakeControlStyle\022!.Ind"
    "yFramework.Protobuf.HRI.Empty\032-.IndyFram"
    "ework.Protobuf.HRI.BrakeControlStyle\"\000\022^"
    "\n\010SetBrake\022\'.IndyFramework.Protobuf.HRI."
    "SetBrakeReq\032\'.IndyFramework.Protobuf.HRI"
    ".SetBrakeRes\"\000\022^\n\010SetServo\022\'.IndyFramewo"
    "rk.Protobuf.HRI.SetServoReq\032\'.IndyFramew"
    "ork.Protobuf.HRI.SetServoRes\"\000\022s\n\017SetAut"
    "oServoOff\022..IndyFramework.Protobuf.HRI.S"
    "etAutoServoOffReq\032..IndyFramework.Protob"
    "uf.HRI.SetAutoServoOffRes\"\000\022s\n\017GetAutoSe"
    "rvoOff\022..IndyFramework.Protobuf.HRI.GetA"
    "utoServoOffReq\032..IndyFramework.Protobuf."
    "HRI.GetAutoServoOffRes\"\000\022W\n\013ExecuteTool\022"
    " .IndyFramework.Protobuf.HRI.Name\032$.Indy"
    "Framework.Protobuf.HRI.Response\"\000\022y\n\021Inv"
    "erseKinematics\0220.IndyFramework.Protobuf."
    "HRI.InverseKinematicsReq\0320.IndyFramework"
    ".Protobuf.HRI.InverseKinematicsRes\"\000\022\205\001\n"
    "\025CalculateRelativePose\0224.IndyFramework.P"
    "rotobuf.HRI.CalculateRelativePoseReq\0324.I"
    "ndyFramework.Protobuf.HRI.CalculateRelat"
    "ivePoseRes\"\000\022\213\001\n\027CalculateCurrentPoseRel"
    "\0226.IndyFramework.Protobuf.HRI.CalculateC"
    "urrentPoseRelReq\0326.IndyFramework.Protobu"
    "f.HRI.CalculateCurrentPoseRelRes\"\000\022a\n\024Se"
    "tManualRecoverMode\022!.IndyFramework.Proto"
    "buf.HRI.State\032$.IndyFramework.Protobuf.H"
    "RI.Response\"\000\022\\\n\017SetServoRecover\022!.IndyF"
    "ramework.Protobuf.HRI.Servo\032$.IndyFramew"
    "ork.Protobuf.HRI.Response\"\000\022^\n\017JogJointR"
    "ecover\022#.IndyFramework.Protobuf.HRI.JogJ"
    "Tar\032$.IndyFramework.Protobuf.HRI.Respons"
    "e\"\000\022[\n\007Recover\022&.IndyFramework.Protobuf."
    "HRI.RecoverReq\032&.IndyFramework.Protobuf."
    "HRI.RecoverRes\"\000\022X\n\006Reboot\022%.IndyFramewo"
    "rk.Protobuf.HRI.RebootReq\032%.IndyFramewor"
    "k.Protobuf.HRI.RebootRes\"\000\022^\n\010PowerOff\022\'"
    ".IndyFramework.Protobuf.HRI.PowerOffReq\032"
    "\'.IndyFramework.Protobuf.HRI.PowerOffRes"
    "\"\000\022l\n\014UpdateIndySW\022+.IndyFramework.Proto"
    "buf.HRI.UpdateIndySWReq\032+.IndyFramework."
    "Protobuf.HRI.UpdateIndySWRes\"\000(\001\022]\n\007GetF"
    "ile\022&.IndyFramework.Protobuf.HRI.GetFile"
    "Req\032&.IndyFramework.Protobuf.HRI.GetFile"
    "Res\"\0000\001\022]\n\007SetFile\022&.IndyFramework.Proto"
    "buf.HRI.SetFileReq\032&.IndyFramework.Proto"
    "buf.HRI.SetFileRes\"\000(\001\022d\n\nRenameFile\022).I"
    "ndyFramework.Protobuf.HRI.RenameFileReq\032"
    ").IndyFramework.Protobuf.HRI.RenameFileR"
    "es\"\000\022d\n\nRemoveFile\022).IndyFramework.Proto"
    "buf.HRI.RemoveFileReq\032).IndyFramework.Pr"
    "otobuf.HRI.RemoveFileRes\"\000\022\177\n\023GetIndexPr"
    "ogramList\0222.IndyFramework.Protobuf.HRI.G"
    "etIndexProgramListReq\0322.IndyFramework.Pr"
    "otobuf.HRI.GetIndexProgramListRes\"\000\022p\n\016G"
    "etProgramList\022-.IndyFramework.Protobuf.H"
    "RI.GetProgramListReq\032-.IndyFramework.Pro"
    "tobuf.HRI.GetProgramListRes\"\000\022d\n\nGetLogL"
    "ist\022).IndyFramework.Protobuf.HRI.GetLogL"
    "istReq\032).IndyFramework.Protobuf.HRI.GetL"
    "ogListRes\"\000\022o\n\rGetLogContent\022,.IndyFrame"
    "work.Protobuf.HRI.GetLogContentReq\032,.Ind"
    "yFramework.Protobuf.HRI.GetLogContentRes"
    "\"\0000\001\022{\n\021GetLogContentList\0220.IndyFramewor"
    "k.Protobuf.HRI.GetLogContentListReq\0320.In"
    "dyFramework.Protobuf.HRI.GetLogContentLi"
    "stRes\"\0000\001\022Z\n\006GetLog\022%.IndyFramework.Prot"
    "obuf.HRI.GetLogReq\032%.IndyFramework.Proto"
    "buf.HRI.GetLogRes\"\0000\001\022s\n\017GetVariableList"
    "\022..IndyFramework.Protobuf.HRI.GetVariabl"
    "eListReq\032..IndyFramework.Protobuf.HRI.Ge"
    "tVariableListRes\"\000\022\221\001\n\031GetMonitoringVari"
    "ableList\0228.IndyFramework.Protobuf.HRI.Ge"
    "tMonitoringVariableListReq\0328.IndyFramewo"
    "rk.Protobuf.HRI.GetMonitoringVariableLis"
    "tRes\"\000\022\221\001\n\031SetMonitoringVariableList\0228.I"
    "ndyFramework.Protobuf.HRI.SetMonitoringV"
    "ariableListReq\0328.IndyFramework.Protobuf."
    "HRI.SetMonitoringVariableListRes\"\000\022|\n\022Ge"
    "tPalletMakerList\0221.IndyFramework.Protobu"
    "f.HRI.GetPalletMakerListReq\0321.IndyFramew"
    "ork.Protobuf.HRI.GetPalletMakerListRes\"\000"
    "\022|\n\022SetPalletMakerList\0221.IndyFramework.P"
    "rotobuf.HRI.SetPalletMakerListReq\0321.Indy"
    "Framework.Protobuf.HRI.SetPalletMakerLis"
    "tRes\"\000\022\216\001\n\030CheckAproachRetractValid\0227.In"
    "dyFramework.Protobuf.HRI.CheckAproachRet"
    "ractValidReq\0327.IndyFramework.Protobuf.HR"
    "I.CheckAproachRetractValidRes\"\000\022|\n\022GetPa"
    "lletPointList\0221.IndyFramework.Protobuf.H"
    "RI.GetPalletPointListReq\0321.IndyFramework"
    ".Protobuf.HRI.GetPalletPointListRes\"\000\022|\n"
    "\022SetPalletPointList\0221.IndyFramework.Prot"
    "obuf.HRI.SetPalletPointListReq\0321.IndyFra"
    "mework.Protobuf.HRI.SetPalletPointListRe"
    "s\"\000\022X\n\013GetConveyor\022!.IndyFramework.Proto"
    "buf.HRI.Empty\032$.IndyFramework.Protobuf.H"
    "RI.Conveyor\"\000\022[\n\017SetConveyorName\022 .IndyF"
    "ramework.Protobuf.HRI.Name\032$.IndyFramewo"
    "rk.Protobuf.HRI.Response\"\000\022]\n\021SetConveyo"
    "rByName\022 .IndyFramework.Protobuf.HRI.Nam"
    "e\032$.IndyFramework.Protobuf.HRI.Response\""
    "\000\022a\n\022SetConveyorEncoder\022#.IndyFramework."
    "Protobuf.HRI.Encoder\032$.IndyFramework.Pro"
    "tobuf.HRI.Response\"\000\022a\n\022SetConveyorTrigg"
    "er\022#.IndyFramework.Protobuf.HRI.Trigger\032"
    "$.IndyFramework.Protobuf.HRI.Response\"\000\022"
    "^\n\021SetConveyorOffset\022!.IndyFramework.Pro"
    "tobuf.HRI.Float\032$.IndyFramework.Protobuf"
    ".HRI.Response\"\000\022g\n\027SetConveyorStartingPo"
    "se\022$.IndyFramework.Protobuf.HRI.PosePair"
    "\032$.IndyFramework.Protobuf.HRI.Response\"\000"
    "\022g\n\027SetConveyorTerminalPose\022$.IndyFramew"
    "ork.Protobuf.HRI.PosePair\032$.IndyFramewor"
    "k.Protobuf.HRI.Response\"\000\022b\n\020GetConveyor"
    "State\022!.IndyFramework.Protobuf.HRI.Empty"
    "\032).IndyFramework.Protobuf.HRI.ConveyorSt"
    "ate\"\000\022o\n\022HoldToMoveConveyor\0221.IndyFramew"
    "ork.Protobuf.HRI.HoldToMoveConveyorReq\032$"
    ".IndyFramework.Protobuf.HRI.Response\"\000\022s"
    "\n\017GetConveyorList\022..IndyFramework.Protob"
    "uf.HRI.GetConveyorListReq\032..IndyFramewor"
    "k.Protobuf.HRI.GetConveyorListRes\"\000\022s\n\017S"
    "etConveyorList\022..IndyFramework.Protobuf."
    "HRI.SetConveyorListReq\032..IndyFramework.P"
    "rotobuf.HRI.SetConveyorListRes\"\000\022d\n\nSetI"
    "nching\022).IndyFramework.Protobuf.HRI.SetI"
    "nchingReq\032).IndyFramework.Protobuf.HRI.S"
    "etInchingRes\"\000\022g\n\013StopInching\022*.IndyFram"
    "ework.Protobuf.HRI.StopInchingReq\032*.Indy"
    "Framework.Protobuf.HRI.StopInchingRes\"\000\022"
    "X\n\006SetArc\022%.IndyFramework.Protobuf.HRI.S"
    "etArcReq\032%.IndyFramework.Protobuf.HRI.Se"
    "tArcRes\"\000\022^\n\010SetTouch\022\'.IndyFramework.Pr"
    "otobuf.HRI.SetTouchReq\032\'.IndyFramework.P"
    "rotobuf.HRI.SetTouchRes\"\000\022X\n\006SetGas\022%.In"
    "dyFramework.Protobuf.HRI.SetGasReq\032%.Ind"
    "yFramework.Protobuf.HRI.SetGasRes\"\000\022a\n\tI"
    "sTouched\022(.IndyFramework.Protobuf.HRI.Is"
    "TouchedReq\032(.IndyFramework.Protobuf.HRI."
    "IsTouchedRes\"\000\022\177\n\023GetVisionServerList\0222."
    "IndyFramework.Protobuf.HRI.GetVisionServ"
    "erListReq\0322.IndyFramework.Protobuf.HRI.G"
    "etVisionServerListRes\"\000\022\177\n\023SetVisionServ"
    "erList\0222.IndyFramework.Protobuf.HRI.SetV"
    "isionServerListReq\0322.IndyFramework.Proto"
    "buf.HRI.SetVisionServerListRes\"\000\022\177\n\023GetV"
    "isionObjectList\0222.IndyFramework.Protobuf"
    ".HRI.GetVisionObjectListReq\0322.IndyFramew"
    "ork.Protobuf.HRI.GetVisionObjectListRes\""
    "\000\022k\n\022GetVisionDetection\022).IndyFramework."
    "Protobuf.HRI.VisionRequest\032(.IndyFramewo"
    "rk.Protobuf.HRI.VisionResult\"\000\022k\n\022GetVis"
    "ionRetrieval\022).IndyFramework.Protobuf.HR"
    "I.VisionRequest\032(.IndyFramework.Protobuf"
    ".HRI.VisionResult\"\000\022|\n\022GetGcodeMotionLis"
    "t\0221.IndyFramework.Protobuf.HRI.GetGcodeM"
    "otionListReq\0321.IndyFramework.Protobuf.HR"
    "I.GetGcodeMotionListRes\"\000\022\177\n\023GetModbusSe"
    "rverList\0222.IndyFramework.Protobuf.HRI.Ge"
    "tModbusServerListReq\0322.IndyFramework.Pro"
    "tobuf.HRI.GetModbusServerListRes\"\000\022\177\n\023Se"
    "tModbusServerList\0222.IndyFramework.Protob"
    "uf.HRI.SetModbusServerListReq\0322.IndyFram"
    "ework.Protobuf.HRI.SetModbusServerListRe"
    "s\"\000\022\205\001\n\025CheckModbusConnection\0224.IndyFram"
    "ework.Protobuf.HRI.CheckModbusConnection"
    "Req\0324.IndyFramework.Protobuf.HRI.CheckMo"
    "dbusConnectionRes\"\000\022v\n\020GetToolFrameList\022"
    "/.IndyFramework.Protobuf.HRI.GetToolFram"
    "eListReq\032/.IndyFramework.Protobuf.HRI.Ge"
    "tToolFrameListRes\"\000\022v\n\020SetToolFrameList\022"
    "/.IndyFramework.Protobuf.HRI.SetToolFram"
    "eListReq\032/.IndyFramework.Protobuf.HRI.Se"
    "tToolFrameListRes\"\000\022s\n\017GetRefFrameList\022."
    ".IndyFramework.Protobuf.HRI.GetRefFrameL"
    "istReq\032..IndyFramework.Protobuf.HRI.GetR"
    "efFrameListRes\"\000\022s\n\017SetRefFrameList\022..In"
    "dyFramework.Protobuf.HRI.SetRefFrameList"
    "Req\032..IndyFramework.Protobuf.HRI.SetRefF"
    "rameListRes\"\000\022v\n\020GetCustomPosList\022/.Indy"
    "Framework.Protobuf.HRI.GetCustomPosListR"
    "eq\032/.IndyFramework.Protobuf.HRI.GetCusto"
    "mPosListRes\"\000\022v\n\020SetCustomPosList\022/.Indy"
    "Framework.Protobuf.HRI.SetCustomPosListR"
    "eq\032/.IndyFramework.Protobuf.HRI.SetCusto"
    "mPosListRes\"\000\022\177\n\023SetJointControlGain\0222.I"
    "ndyFramework.Protobuf.HRI.SetJointContro"
    "lGainReq\0322.IndyFramework.Protobuf.HRI.Se"
    "tJointControlGainRes\"\000\022\177\n\023GetJointContro"
    "lGain\0222.IndyFramework.Protobuf.HRI.GetJo"
    "intControlGainReq\0322.IndyFramework.Protob"
    "uf.HRI.GetJointControlGainRes\"\000\022|\n\022SetTa"
    "skControlGain\0221.IndyFramework.Protobuf.H"
    "RI.SetTaskControlGainReq\0321.IndyFramework"
    ".Protobuf.HRI.SetTaskControlGainRes\"\000\022|\n"
    "\022GetTaskControlGain\0221.IndyFramework.Prot"
    "obuf.HRI.GetTaskControlGainReq\0321.IndyFra"
    "mework.Protobuf.HRI.GetTaskControlGainRe"
    "s\"\000\022\213\001\n\027SetImpedanceControlGain\0226.IndyFr"
    "amework.Protobuf.HRI.SetImpedanceControl"
    "GainReq\0326.IndyFramework.Protobuf.HRI.Set"
    "ImpedanceControlGainRes\"\000\022\213\001\n\027GetImpedan"
    "ceControlGain\0226.IndyFramework.Protobuf.H"
    "RI.GetImpedanceControlGainReq\0326.IndyFram"
    "ework.Protobuf.HRI.GetImpedanceControlGa"
    "inRes\"\000\022\177\n\023SetForceControlGain\0222.IndyFra"
    "mework.Protobuf.HRI.SetForceControlGainR"
    "eq\0322.IndyFramework.Protobuf.HRI.SetForce"
    "ControlGainRes\"\000\022\177\n\023GetForceControlGain\022"
    "2.IndyFramework.Protobuf.HRI.GetForceCon"
    "trolGainReq\0322.IndyFramework.Protobuf.HRI"
    ".GetForceControlGainRes\"\000\022\205\001\n\025SetExtAxis"
    "ControlGain\0224.IndyFramework.Protobuf.HRI"
    ".SetExtAxisControlGainReq\0324.IndyFramewor"
    "k.Protobuf.HRI.SetExtAxisControlGainRes\""
    "\000\022\205\001\n\025GetExtAxisControlGain\0224.IndyFramew"
    "ork.Protobuf.HRI.GetExtAxisControlGainRe"
    "q\0324.IndyFramework.Protobuf.HRI.GetExtAxi"
    "sControlGainRes\"\000\022\243\001\n\'SetSensorlessCompl"
    "ianceControlJointGain\022:.IndyFramework.Pr"
    "otobuf.HRI.SetSensorlessComplianceGainRe"
    "q\032:.IndyFramework.Protobuf.HRI.SetSensor"
    "lessComplianceGainRes\"\000\022\243\001\n\'GetSensorles"
    "sComplianceControlJointGain\022:.IndyFramew"
    "ork.Protobuf.HRI.GetSensorlessCompliance"
    "GainReq\032:.IndyFramework.Protobuf.HRI.Get"
    "SensorlessComplianceGainRes\"\000\022\242\001\n&SetSen"
    "sorlessComplianceControlTaskGain\022:.IndyF"
    "ramework.Protobuf.HRI.SetSensorlessCompl"
    "ianceGainReq\032:.IndyFramework.Protobuf.HR"
    "I.SetSensorlessComplianceGainRes\"\000\022\242\001\n&G"
    "etSensorlessComplianceControlTaskGain\022:."
    "IndyFramework.Protobuf.HRI.GetSensorless"
    "ComplianceGainReq\032:.IndyFramework.Protob"
    "uf.HRI.GetSensorlessComplianceGainRes\"\000\022"
    "g\n\013SetFricComp\022*.IndyFramework.Protobuf."
    "HRI.SetFricCompReq\032*.IndyFramework.Proto"
    "buf.HRI.SetFricCompRes\"\000\022g\n\013GetFricComp\022"
    "*.IndyFramework.Protobuf.HRI.GetFricComp"
    "Req\032*.IndyFramework.Protobuf.HRI.GetFric"
    "CompRes\"\000\022d\n\nSetHomePos\022).IndyFramework."
    "Protobuf.HRI.SetHomePosReq\032).IndyFramewo"
    "rk.Protobuf.HRI.SetHomePosRes\"\000\022d\n\nGetHo"
    "mePos\022).IndyFramework.Protobuf.HRI.GetHo"
    "mePosReq\032).IndyFramework.Protobuf.HRI.Ge"
    "tHomePosRes\"\000\022g\n\013SetMountPos\022*.IndyFrame"
    "work.Protobuf.HRI.SetMountPosReq\032*.IndyF"
    "ramework.Protobuf.HRI.SetMountPosRes\"\000\022g"
    "\n\013GetMountPos\022*.IndyFramework.Protobuf.H"
    "RI.GetMountPosReq\032*.IndyFramework.Protob"
    "uf.HRI.GetMountPosRes\"\000\022s\n\017GetPackagingP"
    "os\022..IndyFramework.Protobuf.HRI.GetPacka"
    "gingPosReq\032..IndyFramework.Protobuf.HRI."
    "GetPackagingPosRes\"\000\022s\n\017SetToolProperty\022"
    "..IndyFramework.Protobuf.HRI.SetToolProp"
    "ertyReq\032..IndyFramework.Protobuf.HRI.Set"
    "ToolPropertyRes\"\000\022s\n\017GetToolProperty\022..I"
    "ndyFramework.Protobuf.HRI.GetToolPropert"
    "yReq\032..IndyFramework.Protobuf.HRI.GetToo"
    "lPropertyRes\"\000\022\213\001\n\027GetOnStartProgramConf"
    "ig\0226.IndyFramework.Protobuf.HRI.GetOnSta"
    "rtProgramConfigReq\0326.IndyFramework.Proto"
    "buf.HRI.GetOnStartProgramConfigRes\"\000\022\213\001\n"
    "\027SetOnStartProgramConfig\0226.IndyFramework"
    ".Protobuf.HRI.SetOnStartProgramConfigReq"
    "\0326.IndyFramework.Protobuf.HRI.SetOnStart"
    "ProgramConfigRes\"\000\022\202\001\n\024GetSafetyLimitCon"
    "fig\0223.IndyFramework.Protobuf.HRI.GetSafe"
    "tyLimitConfigReq\0323.IndyFramework.Protobu"
    "f.HRI.GetSafetyLimitConfigRes\"\000\022\202\001\n\024SetS"
    "afetyLimitConfig\0223.IndyFramework.Protobu"
    "f.HRI.SetSafetyLimitConfigReq\0323.IndyFram"
    "ework.Protobuf.HRI.SetSafetyLimitConfigR"
    "es\"\000\022\177\n\023GetSafetyStopConfig\0222.IndyFramew"
    "ork.Protobuf.HRI.GetSafetyStopConfigReq\032"
    "2.IndyFramework.Protobuf.HRI.GetSafetySt"
    "opConfigRes\"\000\022\177\n\023SetSafetyStopConfig\0222.I"
    "ndyFramework.Protobuf.HRI.SetSafetyStopC"
    "onfigReq\0322.IndyFramework.Protobuf.HRI.Se"
    "tSafetyStopConfigRes\"\000\022\177\n\023GetSafetyConfi"
    "gHash\0222.IndyFramework.Protobuf.HRI.GetSa"
    "fetyConfigHashReq\0322.IndyFramework.Protob"
    "uf.HRI.GetSafetyConfigHashRes\"\000\022h\n\023GetKi"
    "nematicsParams\022!.IndyFramework.Protobuf."
    "HRI.Empty\032,.IndyFramework.Protobuf.HRI.K"
    "inematicsParams\"\000\022[\n\007KeyInit\022&.IndyFrame"
    "work.Protobuf.HRI.KeyInitReq\032&.IndyFrame"
    "work.Protobuf.HRI.KeyInitRes\"\000\022[\n\007KeyDat"
    "a\022&.IndyFramework.Protobuf.HRI.KeyDataRe"
    "q\032&.IndyFramework.Protobuf.HRI.KeyDataRe"
    "s\"\000\022`\n\020ProgramUpdateKey\022!.IndyFramework."
    "Protobuf.HRI.Empty\032\'.IndyFramework.Proto"
    "buf.HRI.ProgramData\"\000\022\210\001\n\030SetDirectTeach"
    "ingModeKey\0224.IndyFramework.Protobuf.HRI."
    "SetDirectTeachingModeReq\0324.IndyFramework"
    ".Protobuf.HRI.SetDirectTeachingModeRes\"\000"
    "\022j\n\021GetAutoModeConfig\022!.IndyFramework.Pr"
    "otobuf.HRI.Empty\0320.IndyFramework.Protobu"
    "f.HRI.GetAutoModeConfigRes\"\000\022g\n\013SetAutoM"
    "ode\022*.IndyFramework.Protobuf.HRI.SetAuto"
    "ModeReq\032*.IndyFramework.Protobuf.HRI.Set"
    "AutoModeRes\"\000\022b\n\rCheckAutoMode\022!.IndyFra"
    "mework.Protobuf.HRI.Empty\032,.IndyFramewor"
    "k.Protobuf.HRI.CheckAutoModeRes\"\000\022h\n\020Che"
    "ckReducedMode\022!.IndyFramework.Protobuf.H"
    "RI.Empty\032/.IndyFramework.Protobuf.HRI.Ch"
    "eckReducedModeRes\"\000\022f\n\017GetReducedRatio\022!"
    ".IndyFramework.Protobuf.HRI.Empty\032..Indy"
    "Framework.Protobuf.HRI.GetReducedRatioRe"
    "s\"\000\022f\n\017GetReducedSpeed\022!.IndyFramework.P"
    "rotobuf.HRI.Empty\032..IndyFramework.Protob"
    "uf.HRI.GetReducedSpeedRes\"\000\022i\n\017SetReduce"
    "dSpeed\022..IndyFramework.Protobuf.HRI.SetR"
    "educedSpeedReq\032$.IndyFramework.Protobuf."
    "HRI.Response\"\000\022Z\n\014GetStopState\022!.IndyFra"
    "mework.Protobuf.HRI.Empty\032%.IndyFramewor"
    "k.Protobuf.HRI.StopState\"\000\022n\n\026GetSafetyF"
    "unctionState\022!.IndyFramework.Protobuf.HR"
    "I.Empty\032/.IndyFramework.Protobuf.HRI.Saf"
    "etyFunctionState\"\000\022p\n\025RequestSafetyFunct"
    "ion\022/.IndyFramework.Protobuf.HRI.SafetyF"
    "unctionState\032$.IndyFramework.Protobuf.HR"
    "I.Response\"\000\022g\n\013PlayProgram\022*.IndyFramew"
    "ork.Protobuf.HRI.PlayProgramReq\032*.IndyFr"
    "amework.Protobuf.HRI.PlayProgramRes\"\000\022v\n"
    "\020PlayIndexProgram\022/.IndyFramework.Protob"
    "uf.HRI.PlayIndexProgramReq\032/.IndyFramewo"
    "rk.Protobuf.HRI.PlayIndexProgramRes\"\000\022s\n"
    "\017PlayProgramLine\022..IndyFramework.Protobu"
    "f.HRI.PlayProgramLineReq\032..IndyFramework"
    ".Protobuf.HRI.PlayProgramLineRes\"\000\022m\n\rRe"
    "sumeProgram\022,.IndyFramework.Protobuf.HRI"
    ".ResumeProgramReq\032,.IndyFramework.Protob"
    "uf.HRI.ResumeProgramRes\"\000\022g\n\013StopProgram"
    "\022*.IndyFramework.Protobuf.HRI.StopProgra"
    "mReq\032*.IndyFramework.Protobuf.HRI.StopPr"
    "ogramRes\"\000\022j\n\014PauseProgram\022+.IndyFramewo"
    "rk.Protobuf.HRI.PauseProgramReq\032+.IndyFr"
    "amework.Protobuf.HRI.PauseProgramRes\"\000\022V"
    "\n\005SetDO\022%.IndyFramework.Protobuf.HRI.DOS"
    "ignals\032$.IndyFramework.Protobuf.HRI.Resp"
    "onse\"\000\022V\n\005SetAO\022%.IndyFramework.Protobuf"
    ".HRI.AOSignals\032$.IndyFramework.Protobuf."
    "HRI.Response\"\000\022a\n\010SetEndDO\022-.IndyFramewo"
    "rk.Protobuf.HRI.EndtoolSignalList\032$.Indy"
    "Framework.Protobuf.HRI.Response\"\000\022Y\n\010Set"
    "EndAO\022%.IndyFramework.Protobuf.HRI.AOSig"
    "nals\032$.IndyFramework.Protobuf.HRI.Respon"
    "se\"\000\022[\n\013SetToolList\022$.IndyFramework.Prot"
    "obuf.HRI.ToolList\032$.IndyFramework.Protob"
    "uf.HRI.Response\"\000\022X\n\013GetToolList\022!.IndyF"
    "ramework.Protobuf.HRI.Empty\032$.IndyFramew"
    "ork.Protobuf.HRI.ToolList\"\000\022c\n\017SetDIConf"
    "igList\022(.IndyFramework.Protobuf.HRI.DICo"
    "nfigList\032$.IndyFramework.Protobuf.HRI.Re"
    "sponse\"\000\022`\n\017GetDIConfigList\022!.IndyFramew"
    "ork.Protobuf.HRI.Empty\032(.IndyFramework.P"
    "rotobuf.HRI.DIConfigList\"\000\022c\n\017SetDOConfi"
    "gList\022(.IndyFramework.Protobuf.HRI.DOCon"
    "figList\032$.IndyFramework.Protobuf.HRI.Res"
    "ponse\"\000\022`\n\017GetDOConfigList\022!.IndyFramewo"
    "rk.Protobuf.HRI.Empty\032(.IndyFramework.Pr"
    "otobuf.HRI.DOConfigList\"\000\022j\n\020SetCollSens"
    "Level\022..IndyFramework.Protobuf.HRI.Colli"
    "sionSensLevel\032$.IndyFramework.Protobuf.H"
    "RI.Response\"\000\022g\n\020GetCollSensLevel\022!.Indy"
    "Framework.Protobuf.HRI.Empty\032..IndyFrame"
    "work.Protobuf.HRI.CollisionSensLevel\"\000\022d"
    "\n\rSetCollPolicy\022+.IndyFramework.Protobuf"
    ".HRI.CollisionPolicy\032$.IndyFramework.Pro"
    "tobuf.HRI.Response\"\000\022a\n\rGetCollPolicy\022!."
    "IndyFramework.Protobuf.HRI.Empty\032+.IndyF"
    "ramework.Protobuf.HRI.CollisionPolicy\"\000\022"
    "k\n\020SetCollSensParam\022/.IndyFramework.Prot"
    "obuf.HRI.CollisionThresholds\032$.IndyFrame"
    "work.Protobuf.HRI.Response\"\000\022h\n\020GetCollS"
    "ensParam\022!.IndyFramework.Protobuf.HRI.Em"
    "pty\032/.IndyFramework.Protobuf.HRI.Collisi"
    "onThresholds\"\000\022^\n\021InitCollSensParam\022!.In"
    "dyFramework.Protobuf.HRI.Empty\032$.IndyFra"
    "mework.Protobuf.HRI.Response\"\000\022q\n\021PlayTu"
    "ningProgram\022).IndyFramework.Protobuf.HRI"
    ".TuningProgram\032/.IndyFramework.Protobuf."
    "HRI.CollisionThresholds\"\000\022`\n\017GetTeleOpDe"
    "vice\022!.IndyFramework.Protobuf.HRI.Empty\032"
    "(.IndyFramework.Protobuf.HRI.TeleOpDevic"
    "e\"\000\022^\n\016GetTeleOpState\022!.IndyFramework.Pr"
    "otobuf.HRI.Empty\032\'.IndyFramework.Protobu"
    "f.HRI.TeleOpState\"\000\022g\n\023ConnectTeleOpDevi"
    "ce\022(.IndyFramework.Protobuf.HRI.TeleOpDe"
    "vice\032$.IndyFramework.Protobuf.HRI.Respon"
    "se\"\000\022c\n\026DisConnectTeleOpDevice\022!.IndyFra"
    "mework.Protobuf.HRI.Empty\032$.IndyFramewor"
    "k.Protobuf.HRI.Response\"\000\022Y\n\017ReadTeleOpI"
    "nput\022!.IndyFramework.Protobuf.HRI.Empty\032"
    "!.IndyFramework.Protobuf.HRI.TeleP\"\000\022[\n\016"
    "StartTeleCalib\022!.IndyFramework.Protobuf."
    "HRI.Empty\032$.IndyFramework.Protobuf.HRI.R"
    "esponse\"\000\022\\\n\017StartTeleRecord\022!.IndyFrame"
    "work.Protobuf.HRI.Empty\032$.IndyFramework."
    "Protobuf.HRI.Response\"\000\022Z\n\rStartTelePlay"
    "\022!.IndyFramework.Protobuf.HRI.Empty\032$.In"
    "dyFramework.Protobuf.HRI.Response\"\000\022Z\n\rS"
    "tartTeleJogL\022!.IndyFramework.Protobuf.HR"
    "I.Empty\032$.IndyFramework.Protobuf.HRI.Res"
    "ponse\"\000\022Z\n\rStartTeleJogJ\022!.IndyFramework"
    ".Protobuf.HRI.Empty\032$.IndyFramework.Prot"
    "obuf.HRI.Response\"\000\022W\n\nStopTeleOp\022!.Indy"
    "Framework.Protobuf.HRI.Empty\032$.IndyFrame"
    "work.Protobuf.HRI.Response\"\000\022_\n\013SetPlayR"
    "ate\022(.IndyFramework.Protobuf.HRI.TelePla"
    "yRate\032$.IndyFramework.Protobuf.HRI.Respo"
    "nse\"\000\022\\\n\013GetPlayRate\022!.IndyFramework.Pro"
    "tobuf.HRI.Empty\032(.IndyFramework.Protobuf"
    ".HRI.TelePlayRate\"\000\022b\n\017GetTeleFileList\022!"
    ".IndyFramework.Protobuf.HRI.Empty\032*.Indy"
    "Framework.Protobuf.HRI.TeleOpFileList\"\000\022"
    "a\n\016SaveTeleMotion\022\'.IndyFramework.Protob"
    "uf.HRI.TeleFileReq\032$.IndyFramework.Proto"
    "buf.HRI.Response\"\000\022a\n\016LoadTeleMotion\022\'.I"
    "ndyFramework.Protobuf.HRI.TeleFileReq\032$."
    "IndyFramework.Protobuf.HRI.Response\"\000\022c\n"
    "\020DeleteTeleMotion\022\'.IndyFramework.Protob"
    "uf.HRI.TeleFileReq\032$.IndyFramework.Proto"
    "buf.HRI.Response\"\000\022]\n\tMoveTeleJ\022(.IndyFr"
    "amework.Protobuf.HRI.MoveTeleJReq\032$.Indy"
    "Framework.Protobuf.HRI.Response\"\000\022]\n\tMov"
    "eTeleL\022(.IndyFramework.Protobuf.HRI.Move"
    "TeleLReq\032$.IndyFramework.Protobuf.HRI.Re"
    "sponse\"\000\022c\n\017SetTeleOpParams\022(.IndyFramew"
    "ork.Protobuf.HRI.TeleOpParams\032$.IndyFram"
    "ework.Protobuf.HRI.Response\"\000\022`\n\017GetTele"
    "OpParams\022!.IndyFramework.Protobuf.HRI.Em"
    "pty\032(.IndyFramework.Protobuf.HRI.TeleOpP"
    "arams\"\000\022g\n\021SetFTSensorConfig\022*.IndyFrame"
    "work.Protobuf.HRI.FTSensorDevice\032$.IndyF"
    "ramework.Protobuf.HRI.Response\"\000\022d\n\021GetF"
    "TSensorConfig\022!.IndyFramework.Protobuf.H"
    "RI.Empty\032*.IndyFramework.Protobuf.HRI.FT"
    "SensorDevice\"\000\022`\n\017GetFTSensorData\022!.Indy"
    "Framework.Protobuf.HRI.Empty\032(.IndyFrame"
    "work.Protobuf.HRI.FTSensorData\"\000\022d\n\016GetL"
    "oadFactors\022!.IndyFramework.Protobuf.HRI."
    "Empty\032-.IndyFramework.Protobuf.HRI.GetLo"
    "adFactorsRes\"\000\022e\n\020SetSanderCommand\022).Ind"
    "yFramework.Protobuf.HRI.SanderCommand\032$."
    "IndyFramework.Protobuf.HRI.Response\"\000\022b\n"
    "\020GetSanderCommand\022!.IndyFramework.Protob"
    "uf.HRI.Empty\032).IndyFramework.Protobuf.HR"
    "I.SanderCommand\"\000\022x\n\027SetSandingStopCondi"
    "tion\0225.IndyFramework.Protobuf.HRI.Sandin"
    "gStopConditionState\032$.IndyFramework.Prot"
    "obuf.HRI.Response\"\000\022u\n\027GetSandingStopCon"
    "dition\022!.IndyFramework.Protobuf.HRI.Empt"
    "y\0325.IndyFramework.Protobuf.HRI.SandingSt"
    "opConditionState\"\000\022^\n\016GetGripperData\022!.I"
    "ndyFramework.Protobuf.HRI.Empty\032\'.IndyFr"
    "amework.Protobuf.HRI.GripperData\"\000\022d\n\021Se"
    "tGripperCommand\022*.IndyFramework.Protobuf"
    ".HRI.GripperCommand\032!.IndyFramework.Prot"
    "obuf.HRI.Empty\"\000\022[\n\016SetSFDActivate\022!.Ind"
    "yFramework.Protobuf.HRI.State\032$.IndyFram"
    "ework.Protobuf.HRI.Response\"\000\022W\n\rIsSFDAc"
    "tivate\022!.IndyFramework.Protobuf.HRI.Empt"
    "y\032!.IndyFramework.Protobuf.HRI.State\"\000\022Z"
    "\n\010SFDLogin\022&.IndyFramework.Protobuf.HRI."
    "SFDAccount\032$.IndyFramework.Protobuf.HRI."
    "Response\"\000\022T\n\nIsSFDLogin\022!.IndyFramework"
    ".Protobuf.HRI.Empty\032!.IndyFramework.Prot"
    "obuf.HRI.State\"\000\022`\n\014SetSFDTarget\022(.IndyF"
    "ramework.Protobuf.HRI.SFDCriTarget\032$.Ind"
    "yFramework.Protobuf.HRI.Response\"\000\022Y\n\014Se"
    "tSFDOption\022!.IndyFramework.Protobuf.HRI."
    "State\032$.IndyFramework.Protobuf.HRI.Respo"
    "nse\"\000\022a\n\016GetSFDProjList\022!.IndyFramework."
    "Protobuf.HRI.Empty\032*.IndyFramework.Proto"
    "buf.HRI.SFDProjectList\"\000\022X\n\tGetSFDCRI\022!."
    "IndyFramework.Protobuf.HRI.Empty\032&.IndyF"
    "ramework.Protobuf.HRI.SFDCriData\"\000\022X\n\013Ge"
    "tTactTime\022!.IndyFramework.Protobuf.HRI.E"
    "mpty\032$.IndyFramework.Protobuf.HRI.TactTi"
    "me\"\000\022r\n\025AddPhotoneoCalibPoint\0224.IndyFram"
    "ework.Protobuf.HRI.AddPhotoneoCalibPoint"
    "Req\032!.IndyFramework.Protobuf.HRI.Empty\"\000"
    "b\006proto3"
};
static const ::_pbi::DescriptorTable* const descriptor_table_hri_2eproto_deps[1] =
    {
        &::descriptor_table_hri_5fmsgs_2eproto,
};
static ::absl::once_flag descriptor_table_hri_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_hri_2eproto = {
    false,
    false,
    21688,
    descriptor_table_protodef_hri_2eproto,
    "hri.proto",
    &descriptor_table_hri_2eproto_once,
    descriptor_table_hri_2eproto_deps,
    1,
    0,
    schemas,
    file_default_instances,
    TableStruct_hri_2eproto::offsets,
    nullptr,
    file_level_enum_descriptors_hri_2eproto,
    file_level_service_descriptors_hri_2eproto,
};

// This function exists to be marked as weak.
// It can significantly speed up compilation by breaking up LLVM's SCC
// in the .pb.cc translation units. Large translation units see a
// reduction of more than 35% of walltime for optimized builds. Without
// the weak attribute all the messages in the file, including all the
// vtables and everything they use become part of the same SCC through
// a cycle like:
// GetMetadata -> descriptor table -> default instances ->
//   vtables -> GetMetadata
// By adding a weak function here we break the connection from the
// individual vtables back into the descriptor table.
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_hri_2eproto_getter() {
  return &descriptor_table_hri_2eproto;
}
// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2
static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_hri_2eproto(&descriptor_table_hri_2eproto);
namespace IndyFramework {
namespace Protobuf {
namespace HRI {
// @@protoc_insertion_point(namespace_scope)
}  // namespace HRI
}  // namespace Protobuf
}  // namespace IndyFramework
namespace google {
namespace protobuf {
}  // namespace protobuf
}  // namespace google
// @@protoc_insertion_point(global_scope)
#include "google/protobuf/port_undef.inc"
